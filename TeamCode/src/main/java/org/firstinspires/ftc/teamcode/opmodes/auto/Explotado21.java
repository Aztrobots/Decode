package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.AutoRobotFast;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.tools.PoseStorage;

import com.pedropathing.geometry.BezierCurve;

@Autonomous(name = "21 NO USAR", group = "!")
public class Explotado21 extends OpMode {

    // ── Field Constants ──────────────────────────────────────────────────────
    private static final Pose START_POSE    = new Pose(123.000, 122,   Math.toRadians(37));
    private static final Pose GATE_POSE     = new Pose(135.2,   56,    Math.toRadians(37));
    private static final Pose SHOOT_END     = new Pose(90.625,  75.078, 0);
    private static final Pose GATE_ORIGIN_0 = new Pose(93.734,  78.328, 0);

    private static final long   DRIVE_SETTLE_MS = 30;
    private static final int    GATE_CYCLES     = 4;
    private static final double SHOOT_TRIGGER_T = 0.80;

    // ── Gate Approach Tuning ─────────────────────────────────────────────────
    // t donde el path Gate baja a power reducido: llegada lenta y directa.
    private static final double GATE_APPROACH_T     = 0.65;
    private static final double GATE_APPROACH_POWER = 0.35;
    private static final long   GATE_DWELL_MS       = 1200;   // contacto REAL — bajar tras verificar SettleMs

    // Constraints del path Gate: el follower NO suelta isBusy hasta estar
    // físicamente settled (vel < 3 in/s, error < 1.0 in, heading < ~1.7°),
    // con 450 ms de ventana de corrección en vez de los 100 ms default.
    // Firma: (tValue, velocity, translational, heading, timeout, brakingStrength, searchLimit, brakingStart)
    private static final PathConstraints GATE_END_CONSTRAINTS =
            new PathConstraints(0.99, 3.0, 1.0, 0.03, 450, 1, 10, 1);

    // ── Objects ──────────────────────────────────────────────────────────────
    private Follower              follower;
    private AutoRobotFast         robot;
    private GoBildaPinpointDriver pinpoint;

    // ── Paths ────────────────────────────────────────────────────────────────
    private PathChain PreLoad;
    private PathChain Stack1;
    private PathChain Shoot;
    private PathChain Stack2;
    private PathChain Shoot2;

    private final PathChain[] Gate          = new PathChain[GATE_CYCLES];
    private final PathChain[] ShootFromGate = new PathChain[GATE_CYCLES];

    // ── FSM ──────────────────────────────────────────────────────────────────
    private enum AutoState {
        PRELOAD,        SHOOT_PRELOAD,
        STACK1,
        BACK_TO_SHOOT,  SHOOT,
        STACK2,
        BACK_TO_SHOOT2, SHOOT2,
        TO_GATE,        GATE_HOLD,
        BACK_FROM_GATE,
        DONE
    }

    private AutoState state        = AutoState.PRELOAD;
    private int       gateCycle    = 0;
    private boolean   shotFired    = false;
    private boolean   pathEndArmed = false;

    private final ElapsedTime matchTimer = new ElapsedTime();

    // ── Instrumentación ──────────────────────────────────────────────────────
    private long settleStartMs   = 0;   // primer atParametricEnd en TO_GATE
    private long lastSettleMs    = 0;   // duración del settle del gate (último ciclo)
    private long pathEndStampMs  = 0;   // atEnd en BACK_FROM_GATE
    private long lastBurstLagMs  = 0;   // cuánto esperó parado a isFinished()

    // ── Init ─────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.usePredictiveBraking = true;

        robot = new AutoRobotFast(this);
        robot.init();
        robot.turret.setMode(TurretSubsystem.TurretMode.AIMBOT);

        buildPaths();

        telemetry.addLine("21 NO USAR [RED] — Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        robot.turret.update();

        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Turret atTarget", robot.turret.atTarget());
        telemetry.addData("Turret Ticks",    robot.turret.getCurrentTicks());
        telemetry.update();
    }

    // ── Start ────────────────────────────────────────────────────────────────
    @Override
    public void start() {
        matchTimer.reset();
        robot.start();
        shooter().state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter().setTargetVelocity(AutoRobotFast.SHOOT_FAR_TPS);

        followFast(PreLoad);

        phaseEntryTime = System.currentTimeMillis();
        state = AutoState.PRELOAD;
    }

    // ── Loop ─────────────────────────────────────────────────────────────────
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
        telemetry.addData("At Speed",     robot.shooter.isAtSpeed());
        telemetry.addData("BurstState",   robot.shooter.getShootState());
        telemetry.addData("isBusy",       follower.isBusy());
        telemetry.addData("atParamEnd",   follower.atParametricEnd());
        // ── Métricas de tuning ──
        telemetry.addData("SettleMs",     lastSettleMs);    // bajar GATE_APPROACH_T/POWER si crece
        telemetry.addData("BurstLagMs",   lastBurstLagMs);  // si >0 estable, adelantar SHOOT_TRIGGER_T
        telemetry.addData("Time (s)",     matchTimer.seconds());
        telemetry.update();
    }

    // ── FSM ──────────────────────────────────────────────────────────────────
    private void runFSM() {
        switch (state) {

            case PRELOAD:
                if (atEnd()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    transition(AutoState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (!shotFired) {
                    robot.shootCommand = robot.buildShootCommand(AutoRobotFast.SHOOT_NEAR_TPS);
                    robot.shootCommand.start();
                    shotFired = true;
                } else if (robot.shootCommand.isFinished()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    followFast(Stack1);
                    transition(AutoState.STACK1);
                }
                break;

            case STACK1:
                if (atEnd()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(127.531, 83.594, Math.toRadians(0))),
                            Math.toRadians(0));
                    transition(AutoState.BACK_TO_SHOOT);
                }
                break;

            case BACK_TO_SHOOT:
                if (inPhaseFor(300)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    followFast(Shoot);
                    transition(AutoState.SHOOT);
                }
                break;

            case SHOOT:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobotFast.SHOOT_NEAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        followFast(Stack2);
                        transition(AutoState.STACK2);
                    }
                }
                break;

            case STACK2:
                if (atEnd()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(126.703, 59.484, Math.toRadians(0))),
                            Math.toRadians(0));
                    transition(AutoState.BACK_TO_SHOOT2);
                }
                break;

            case BACK_TO_SHOOT2:
                if (inPhaseFor(300)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    followFast(Shoot2);
                    transition(AutoState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobotFast.SHOOT_NEAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
                        gateCycle = 0;
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        followFast(Gate[0]);
                        transition(AutoState.TO_GATE);
                    }
                }
                break;

            case TO_GATE:
                // Instrumentación: timestamp del primer atParametricEnd
                if (settleStartMs == 0 && follower.atParametricEnd()) {
                    settleStartMs = System.currentTimeMillis();
                }
                // Settle FÍSICO: isBusy baja cuando vel/error/heading cumplen
                // GATE_END_CONSTRAINTS. El holdEnd=true ya engancha holdPoint
                // automáticamente — no llamar holdPoint manual (su breakFollowing
                // resetea PIDFs a mitad del settle).
                if (pathEndArmed && !follower.isBusy()) {
                    lastSettleMs  = (settleStartMs > 0)
                            ? System.currentTimeMillis() - settleStartMs : 0;
                    settleStartMs = 0;
                    transition(AutoState.GATE_HOLD);   // dwell arranca YA en contacto
                }
                break;

            case GATE_HOLD:
                if (inPhaseFor(GATE_DWELL_MS)) {
                    // followFast restaura setMaxPower(1.0) — el callback del Gate
                    // lo dejó en GATE_APPROACH_POWER (setMaxPower es global).
                    followFast(ShootFromGate[gateCycle]);
                    transition(AutoState.BACK_FROM_GATE);
                }
                break;

            case BACK_FROM_GATE:
                // shootCommand arrancó vía ParametricCallback en t=SHOOT_TRIGGER_T.
                if (pathEndStampMs == 0 && atEnd()) {
                    pathEndStampMs = System.currentTimeMillis();
                }
                if (atEnd() && robot.shootCommand.isFinished()) {
                    lastBurstLagMs = (pathEndStampMs > 0)
                            ? System.currentTimeMillis() - pathEndStampMs : 0;
                    pathEndStampMs = 0;

                    gateCycle++;
                    if (gateCycle < GATE_CYCLES) {
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        followFast(Gate[gateCycle]);
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

    // ── Path Builder ─────────────────────────────────────────────────────────
    private void buildPaths() {

        PreLoad = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.000, 121.500),
                        new Pose(98.5,    84)))
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();

        Stack1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(98.5,    83.563),
                        new Pose(115.531, 84)))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        Shoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(115.531, 82),
                        new Pose(98.063,  80.906)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Stack2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(98.063,  80.906),
                        new Pose(94.211,  56.641),
                        new Pose(126.703, 57.484)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(126.703, 57.484),
                        new Pose(93.734,  78.328)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Gate[0] = follower.pathBuilder()
                .addPath(new BezierLine(GATE_ORIGIN_0, GATE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), GATE_POSE.getHeading())
                .addParametricCallback(GATE_APPROACH_T,
                        () -> follower.setMaxPower(GATE_APPROACH_POWER))
                .build();

        for (int i = 1; i < GATE_CYCLES; i++) {
            Gate[i] = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_END, GATE_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(0), GATE_POSE.getHeading())
                    .addParametricCallback(GATE_APPROACH_T,
                            () -> follower.setMaxPower(GATE_APPROACH_POWER))
                    .build();
        }

        // Constraints de settle físico SOLO en los paths Gate
        for (int i = 0; i < GATE_CYCLES; i++) {
            Gate[i].setConstraintsForAll(GATE_END_CONSTRAINTS);
        }

        for (int i = 0; i < GATE_CYCLES; i++) {
            ShootFromGate[i] = follower.pathBuilder()
                    .addPath(new BezierLine(GATE_POSE, SHOOT_END))
                    .setLinearHeadingInterpolation(GATE_POSE.getHeading(), Math.toRadians(0))
                    .addParametricCallback(SHOOT_TRIGGER_T, () -> {
                        robot.shootCommand = robot.buildShootCommand(AutoRobotFast.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    })
                    .build();
        }
    }

    // ── Utils ────────────────────────────────────────────────────────────────
    private long phaseEntryTime = 0;

    /** Lanza un path a full power. Centraliza el reset del setMaxPower global. */
    private void followFast(PathChain chain) {
        follower.setMaxPower(1.0);
        follower.followPath(chain, true);
    }

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