package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tools.AutoRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;


@Disabled
@Autonomous(name = "18 Close S3 [RED]", group = "!")
public class Auto18CloseS3 extends OpMode {


    private static final Pose START_POSE  = new Pose(128.500, 111.000, Math.toRadians(-90));
    private static final Pose SHOOT_POSE  = new Pose(90.844,   76.922, Math.toRadians(0));
    private static final Pose GATE_TAKE   = new Pose(136.5,    58.5,   Math.toRadians(40));
    private static final Pose STACK3_POSE = new Pose(120.297,  35.000, Math.toRadians(-90));


    private Follower  follower;
    private AutoRobot robot;

    // ─── PATHS ───────────────────────────────────────────────────────────────
    private PathChain pathPreLoad;
    private PathChain pathStack1;
    private PathChain pathShoot;
    private PathChain pathStack2;
    private PathChain pathShoot1;
    private PathChain pathTake1;
    private PathChain pathShoot2;
    private PathChain pathStack3;
    private PathChain pathShoot3;

    // ─── FSM ─────────────────────────────────────────────────────────────────
    private enum AutoState {
        PRELOAD_DRIVE, PRELOAD_EXEC,   // ← dispara precargadas antes de ir a Stack1
        STACK1_DRIVE,
        SHOOT_DRIVE,   SHOOT_EXEC,
        STACK2_DRIVE,
        SHOOT1_DRIVE,  SHOOT1_EXEC,
        TAKE1_DRIVE,   TAKE1_HOLD,
        SHOOT2_DRIVE,  SHOOT2_EXEC,
        // STACK3_DRIVE, STACK3_HOLD,
        // SHOOT3_DRIVE, SHOOT3_EXEC,
        DONE
    }

    private AutoState state = AutoState.PRELOAD_DRIVE;
    private final ElapsedTime matchTimer = new ElapsedTime();

    // ─── INIT ────────────────────────────────────────────────────────────────
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

        buildPaths();

        telemetry.addLine("Auto18CloseS3 [RED] — Ready");
        telemetry.update();
    }

    // ─── START ───────────────────────────────────────────────────────────────
    @Override
    public void start() {
        matchTimer.reset();
        robot.start();
        shooter().state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter().setTargetVelocity(AutoRobot.SHOOT_FAR_TPS);

        // Intake lento durante el drive inicial — no satura el mecanismo en movimiento.
        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING_SLOW);

        follower.followPath(pathPreLoad, true);
        state = AutoState.PRELOAD_DRIVE;
    }

    // ─── LOOP ────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        follower.update();
        robot.update();

        runFSM();

        telemetry.addData("State",        state);
        telemetry.addData("Pose",         follower.getPose().toString());
        telemetry.addData("FW TPS",       robot.shooter.getCurrentTPS());
        telemetry.addData("FW Target",    robot.shooter.getTargetTPS());
        telemetry.addData("Turret tck",   robot.turret.getCurrentTicks());
        telemetry.addData("At Speed",     robot.shooter.isAtSpeed());
        telemetry.addData("ShootCmd run", robot.shootCommand.isRunning());
        telemetry.addData("ShootCmd fin", robot.shootCommand.isFinished());
        telemetry.addData("BurstState",   robot.shooter.getShootState());
        telemetry.addData("atParamEnd",   follower.atParametricEnd());
        telemetry.addData("Time (s)",     matchTimer.seconds());
        telemetry.update();
    }

    // ─── FSM ─────────────────────────────────────────────────────────────────
    private void runFSM() {
        switch (state) {

            // Navega al primer shoot pose cargando lento.
            case PRELOAD_DRIVE:
                if (follower.atParametricEnd()) {
                    transition(AutoState.PRELOAD_EXEC);
                }
                break;

            // Dispara precargadas, luego va a Stack1 con intake COLLECTING.
            case PRELOAD_EXEC:
                if (!robot.shootCommand.isRunning() && !robot.shootCommand.isFinished()) {
                    robot.shootCommand.start();
                } else if (robot.shootCommand.isFinished()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    follower.followPath(pathStack1, true);
                    transition(AutoState.STACK1_DRIVE);
                }
                break;

            // HoldPoint en Stack1 — intake COLLECTING ya activo desde PRELOAD_EXEC.
            case STACK1_DRIVE:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(121.000, 86.453, Math.toRadians(-90))),
                            Math.toRadians(-90));
                    transition(AutoState.SHOOT_DRIVE);
                }
                break;

            // Timer de espera en stack; luego navega al shoot pose con intake SLOW.
            case SHOOT_DRIVE:
                if (inPhaseFor(1000)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING_SLOW);
                    follower.followPath(pathShoot, true);
                    transition(AutoState.SHOOT_EXEC);
                }
                break;

            case SHOOT_EXEC:
                if (follower.atParametricEnd()) {
                    if (!robot.shootCommand.isRunning() && !robot.shootCommand.isFinished()) {
                        robot.shootCommand.start();
                    } else if (robot.shootCommand.isFinished()) {
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        follower.followPath(pathStack2, true);
                        transition(AutoState.STACK2_DRIVE);
                    }
                }
                break;

            // HoldPoint en Stack2.
            case STACK2_DRIVE:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(121.534, 60.450, Math.toRadians(-90))),
                            Math.toRadians(-90));
                    transition(AutoState.SHOOT1_DRIVE);
                }
                break;

            case SHOOT1_DRIVE:
                if (inPhaseFor(1000)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING_SLOW);
                    follower.followPath(pathShoot1, true);
                    transition(AutoState.SHOOT1_EXEC);
                }
                break;

            case SHOOT1_EXEC:
                if (follower.atParametricEnd()) {
                    if (!robot.shootCommand.isRunning() && !robot.shootCommand.isFinished()) {
                        robot.shootCommand.start();
                    } else if (robot.shootCommand.isFinished()) {
                        // Directo a GATE_TAKE — sin approach intermedio.
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        follower.followPath(pathTake1, true);
                        transition(AutoState.TAKE1_DRIVE);
                    }
                }
                break;

            // HoldPoint en GATE_TAKE — intake COLLECTING ya activo.
            case TAKE1_DRIVE:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE), GATE_TAKE.getHeading());
                    transition(AutoState.TAKE1_HOLD);
                }
                break;

            case TAKE1_HOLD:
                if (inPhaseFor(1000)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING_SLOW);
                    follower.followPath(pathShoot2, true);
                    transition(AutoState.SHOOT2_DRIVE);
                }
                break;

            case SHOOT2_DRIVE:
                if (follower.atParametricEnd()) {
                    if (!robot.shootCommand.isRunning() && !robot.shootCommand.isFinished()) {
                        robot.shootCommand.start();
                        transition(AutoState.SHOOT2_EXEC);
                    }
                }
                break;

            case SHOOT2_EXEC:
                if (robot.shootCommand.isFinished()) {
                    // ── Descomentar para activar Stack3 ──────────────────────
                    // robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    // follower.followPath(pathStack3, true);
                    // transition(AutoState.STACK3_DRIVE);
                    transition(AutoState.DONE);
                }
                break;

            // ── Stack3 variante ──────────────────────────────────────────────
            //
            // case STACK3_DRIVE:
            //     if (follower.atParametricEnd()) {
            //         follower.holdPoint(new BezierPoint(STACK3_POSE), STACK3_POSE.getHeading());
            //         transition(AutoState.STACK3_HOLD);
            //     }
            //     break;
            //
            // case STACK3_HOLD:
            //     if (inPhaseFor(2000)) {
            //         robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING_SLOW);
            //         follower.followPath(pathShoot3, true);
            //         transition(AutoState.SHOOT3_DRIVE);
            //     }
            //     break;
            //
            // case SHOOT3_DRIVE:
            //     if (follower.atParametricEnd()) {
            //         if (!robot.shootCommand.isRunning() && !robot.shootCommand.isFinished()) {
            //             robot.shootCommand.start();
            //             transition(AutoState.SHOOT3_EXEC);
            //         }
            //     }
            //     break;
            //
            // case SHOOT3_EXEC:
            //     if (robot.shootCommand.isFinished()) {
            //         transition(AutoState.DONE);
            //     }
            //     break;

            case DONE:
                robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                shooter().setTargetVelocity(0);
                shooter().state = ShooterSubsystem.ShooterState.SHOOTER_OFF;
                robot.turret.setMode(TurretSubsystem.TurretMode.HOLD);
                break;
        }
    }

    // ─── BUILD PATHS ─────────────────────────────────────────────────────────
    private void buildPaths() {

        pathPreLoad = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, new Pose(111.000, 94.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        pathStack1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(111.000, 94.000),
                        new Pose(118.000, 94.000),
                        new Pose(121.000, 86.453)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        pathShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(121.000, 86.453),
                        new Pose(110.000, 94.000)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        pathStack2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(110.000, 94.000),
                        new Pose(119.000, 82.750),
                        new Pose(121.534, 60.450)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        pathShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(121.534, 60.450),
                        SHOOT_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        // Directo de SHOOT_POSE a GATE_TAKE.
        pathTake1 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, GATE_TAKE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        pathShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_TAKE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        pathStack3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        SHOOT_POSE,
                        new Pose(123.016, 64.133),
                        STACK3_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        pathShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(STACK3_POSE, SHOOT_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
    }

    // ─── UTILIDADES ──────────────────────────────────────────────────────────
    private long phaseEntryTime = 0;

    private void transition(AutoState newState) {
        state          = newState;
        phaseEntryTime = System.currentTimeMillis();
    }

    private boolean inPhaseFor(long ms) {
        return (System.currentTimeMillis() - phaseEntryTime) >= ms;
    }

    private ShooterSubsystem shooter() { return robot.shooter; }
}