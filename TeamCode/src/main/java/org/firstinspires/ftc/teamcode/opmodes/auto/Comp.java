package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.AutoRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.tools.PoseStorage;

@Autonomous(name = "18 SEGURO", group = "!")
public class Comp extends OpMode {

    // ── Field Constants ──────────────────────────────────────────────────────
    private static final Pose START_POSE = new Pose(123.000, 122,    Math.toRadians(37));
    private static final Pose GATE_POSE  = new Pose(138,   53.5,   Math.toRadians(38));
    private static final Pose GATE_MID   = new Pose(110.734, 38.289, 0);
    private static final Pose GATE_END   = new Pose(125.547, 58.438, 0);
    private static final Pose SHOOT_END  = new Pose(88.625,  75.078, 0);

    // Origen nominal del Gate en el ciclo 0 (fin de Shoot2).
    // Los ciclos 1 y 2 usan Gate1 y Gate2 con orígenes desplazados
    // que aproximan la pose real tras ShootFromGate.
    private static final Pose GATE_ORIGIN_0 = new Pose(93.734, 78.328, 0);

    private static final long DRIVE_SETTLE_MS = 40;
    private static final int  GATE_CYCLES     = 3;

    // ── Objects ───────────────────────────────────────────────────────────────
    private Follower  follower;
    private AutoRobot robot;

    // ── Paths ─────────────────────────────────────────────────────────────────
    // Todos los PathChain están preallocados en buildPaths() durante init().
    // NINGÚN new PathChain ocurre en runtime para evitar OOM en el Control Hub.
    private PathChain PreLoad;
    private PathChain Stack1;
    private PathChain Shoot;
    private PathChain Stack2;
    private PathChain Shoot2;

    // Gate[i] y ShootFromGate[i] corresponden al ciclo i (0, 1, 2).
    // Gate[0]: sale desde Shoot2. Gate[1..2]: salen desde SHOOT_END con heading 0,
    // que es donde termina ShootFromGate con holdEnd=true.
    private final PathChain[] Gate         = new PathChain[GATE_CYCLES];
    private final PathChain[] ShootFromGate = new PathChain[GATE_CYCLES];

    // ── FSM ──────────────────────────────────────────────────────────────────
    private enum AutoState {
        PRELOAD,        SHOOT_PRELOAD,
        STACK1,
        BACK_TO_SHOOT,  SHOOT,
        STACK2,
        BACK_TO_SHOOT2, SHOOT2,
        TO_GATE,        GATE_HOLD,
        BACK_FROM_GATE, SHOOT_GATE,
        DONE
    }

    private AutoState state        = AutoState.PRELOAD;
    private int       gateCycle    = 0;
    private boolean   shotFired    = false;
    private boolean   pathEndArmed = false;

    private final ElapsedTime matchTimer = new ElapsedTime();

    // ── Init ─────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try { Thread.sleep(500); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.usePredictiveBraking = true;

        robot = new AutoRobot(this);
        robot.init();
        robot.turret.setMode(TurretSubsystem.TurretMode.AIMBOT);

        buildPaths();   // toda la allocación ocurre aquí, en init(), no en loop()

        telemetry.addLine("Auto18CloseS3 v4 [RED] — Ready");
        telemetry.update();
    }

    // ── Start ─────────────────────────────────────────────────────────────────
    @Override
    public void start() {
        matchTimer.reset();
        robot.start();
        shooter().state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter().setTargetVelocity(AutoRobot.SHOOT_FAR_TPS);
        follower.followPath(PreLoad, true);
        state = AutoState.PRELOAD;
    }

    // ── Loop ──────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        PoseStorage.currentPose = follower.getPose();
        follower.update();
        robot.update();

        if (!pathEndArmed && inPhaseFor(DRIVE_SETTLE_MS)) {
            pathEndArmed = true;
        }

        runFSM();

        telemetry.addData("State",        state);
        telemetry.addData("Gate Cycle",   gateCycle + "/" + GATE_CYCLES);
        telemetry.addData("Pose",         follower.getPose().toString());
        telemetry.addData("FW TPS",       robot.shooter.getCurrentTPS());
        telemetry.addData("FW Target",    robot.shooter.getTargetTPS());
        telemetry.addData("Turret tck",   robot.turret.getCurrentTicks());
        telemetry.addData("At Speed",     robot.shooter.isAtSpeed());
        telemetry.addData("ShootCmd run", robot.shootCommand.isRunning());
        telemetry.addData("ShootCmd fin", robot.shootCommand.isFinished());
        telemetry.addData("BurstState",   robot.shooter.getShootState());
        telemetry.addData("atParamEnd",   follower.atParametricEnd());
        telemetry.addData("isBusy",       follower.isBusy());
        telemetry.addData("shotFired",    shotFired);
        telemetry.addData("pathEndArmed", pathEndArmed);
        telemetry.addData("Time (s)",     matchTimer.seconds());
        telemetry.update();
    }

    // ── FSM ───────────────────────────────────────────────────────────────────
    private void runFSM() {
        switch (state) {

            // ── Pre-load disc ────────────────────────────────────────────────
            case PRELOAD:
                if (atEnd()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    transition(AutoState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (!shotFired) {
                    robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_NEAR_TPS);
                    robot.shootCommand.start();
                    shotFired = true;
                } else if (robot.shootCommand.isFinished()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    follower.followPath(Stack1, true);
                    transition(AutoState.STACK1);
                }
                break;

            // ── First stack ──────────────────────────────────────────────────
            case STACK1:
                if (atEnd()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(127.531, 83.594, Math.toRadians(0))),
                            Math.toRadians(0));
                    transition(AutoState.BACK_TO_SHOOT);
                }
                break;

            case BACK_TO_SHOOT:
                if (inPhaseFor(500)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(Shoot, true);
                    transition(AutoState.SHOOT);
                }
                break;

            case SHOOT:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_NEAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        follower.followPath(Stack2, true);
                        transition(AutoState.STACK2);
                    }
                }
                break;

            // ── Second stack ─────────────────────────────────────────────────
            case STACK2:
                if (atEnd()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(126.703, 59.484, Math.toRadians(0))),
                            Math.toRadians(0));
                    transition(AutoState.BACK_TO_SHOOT2);
                }
                break;

            case BACK_TO_SHOOT2:
                if (inPhaseFor(500)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(Shoot2, true);
                    transition(AutoState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_NEAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
                        gateCycle = 0;
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        follower.followPath(Gate[0], true);
                        transition(AutoState.TO_GATE);
                    }
                }
                break;

            // ── Gate cycle ────────────────────────────────────────────────────
            case TO_GATE:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_POSE), GATE_POSE.getHeading());
                    transition(AutoState.GATE_HOLD);
                }
                break;

            case GATE_HOLD:
                if (inPhaseFor(1500)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    follower.followPath(ShootFromGate[gateCycle], true);
                    transition(AutoState.BACK_FROM_GATE);
                }
                break;

            case BACK_FROM_GATE:
                if (atEnd()) {
                    transition(AutoState.SHOOT_GATE);
                }
                break;

            case SHOOT_GATE:
                if (!shotFired) {
                    robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                    robot.shootCommand.start();
                    shotFired = true;
                } else if (robot.shootCommand.isFinished()) {
                    gateCycle++;
                    if (gateCycle < GATE_CYCLES) {
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        follower.followPath(Gate[gateCycle], true);
                        transition(AutoState.TO_GATE);
                    } else {
                        transition(AutoState.DONE);
                    }
                }
                break;

            case DONE:
                break;
        }
    }

    // ── Path Builder ──────────────────────────────────────────────────────────
    // REGLA: ningún `new PathChain`, `new BezierCurve`, `new BezierLine` ni
    // `follower.pathBuilder().build()` fuera de este método.
    private void buildPaths() {

        PreLoad = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.000, 121.500),
                        new Pose(97.563,   83.563)))
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();

        Stack1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(97.563,  83.563),
                        new Pose(115.531, 82)))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        Shoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(115.531, 82),
                        new Pose(97.063,  80.906)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Stack2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(97.063, 80.906),
                        new Pose(94.211, 56.641),
                        new Pose(126.703, 57.484)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(126.703, 57.484),
                        new Pose(93.734,  78.328)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // ── Gate[0]: origen = fin de Shoot2 ──────────────────────────────────
        Gate[0] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        GATE_ORIGIN_0,
                        GATE_MID,
                        GATE_END))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(33))
                .build();

        // ── Gate[1] y Gate[2]: origen = SHOOT_END (donde holdEnd ancla el robot
        //    al final de ShootFromGate[i-1]). Heading inicial = 0 porque
        //    ShootFromGate termina con heading 0 en todos los ciclos.
        for (int i = 1; i < GATE_CYCLES; i++) {
            Gate[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_END,
                            GATE_MID,
                            GATE_END))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(33))
                    .build();
        }

        // ── ShootFromGate[0..2]: origen = GATE_END (donde holdPoint ancla el
        //    robot en GATE_HOLD). Heading inicial = 35° (heading de GATE_POSE).
        for (int i = 0; i < GATE_CYCLES; i++) {
            ShootFromGate[i] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            GATE_END,
                            SHOOT_END))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                    .build();
        }
    }

    // ── Utils ─────────────────────────────────────────────────────────────────
    private long phaseEntryTime = 0;

    private void transition(AutoState newState) {
        state          = newState;
        phaseEntryTime = System.currentTimeMillis();
        shotFired      = false;
        pathEndArmed   = false;
    }

    private boolean atEnd() {
        return pathEndArmed && follower.atParametricEnd();
    }

    private boolean inPhaseFor(long ms) {
        return (System.currentTimeMillis() - phaseEntryTime) >= ms;
    }

    private ShooterSubsystem shooter() { return robot.shooter; }
}