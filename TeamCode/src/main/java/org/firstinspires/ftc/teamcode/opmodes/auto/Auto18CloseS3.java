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


@Autonomous(name = "18 Close S3 [RED]", group = "!")
public class Auto18CloseS3 extends OpMode {

    // Field Constants
    private static final Pose START_POSE  = new Pose(123.781, 121.688, Math.toRadians(35));
    private static final Pose SHOOT_POSE  = new Pose(99.984,   83.672, Math.toRadians(0));
    private static final Pose GATE_TAKE1  = new Pose(134.000,  60.000, Math.toRadians(35));
    private static final Pose GATE_TAKE2  = new Pose(134.000,  60.000, Math.toRadians(35));
    private static final Pose GATE_TAKE3  = new Pose(134.000,  60.000, Math.toRadians(35));

    private static final long DRIVE_SETTLE_MS = 80;

    // Objects
    private Follower  follower;
    private AutoRobot robot;

    // Paths
    private PathChain PreLoad;
    private PathChain Stack1;
    private PathChain Shoot;
    private PathChain Stack2;
    private PathChain Shoot1;
    private PathChain toGate1;
    private PathChain shootFromGate1;
    private PathChain toGate2;
    private PathChain shootFromGate2;
    private PathChain toGate3;
    private PathChain shootFromGate3;

    // FSM
    private enum AutoState {
        PRELOAD,        SHOOT_PRELOAD,
        STACK1,
        BACK_TO_SHOOT,  SHOOT,
        STACK2,
        BACK_TO_SHOOT1, SHOOT1,
        GATE1,    GATE1_HOLD,  BACK_FROM_GATE1,  SHOOTGATE1,
        GATE2,    GATE2_HOLD,  BACK_FROM_GATE2,  SHOOTGATE2,
        GATE3,    GATE3_HOLD,  BACK_FROM_GATE3,  SHOOTGATE3,
        DONE
    }

    private AutoState state = AutoState.PRELOAD;
    private final ElapsedTime matchTimer = new ElapsedTime();

    private boolean shotFired    = false;
    private boolean pathEndArmed = false;

    // Init
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

    // Start
    @Override
    public void start() {
        matchTimer.reset();
        robot.start();
        shooter().state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter().setTargetVelocity(AutoRobot.SHOOT_FAR_TPS);
        follower.followPath(PreLoad, true);
        state = AutoState.PRELOAD;
    }

    // Loop
    @Override
    public void loop() {
        follower.update();
        robot.update();

        if (!pathEndArmed && inPhaseFor(DRIVE_SETTLE_MS)) {
            pathEndArmed = true;
        }

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
        telemetry.addData("isBusy",       follower.isBusy());
        telemetry.addData("shotFired",    shotFired);
        telemetry.addData("pathEndArmed", pathEndArmed);
        telemetry.addData("Time (s)",     matchTimer.seconds());
        telemetry.update();
    }

    // Helpers
    private boolean atEnd() {
        return pathEndArmed && follower.atParametricEnd();
    }

    // FSM
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
                    robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_NEAR_TPS);
                    robot.shootCommand.start();
                    shotFired = true;
                } else if (robot.shootCommand.isFinished()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    follower.followPath(Stack1, true);
                    transition(AutoState.STACK1);
                }
                break;

            case STACK1:
                if (atEnd()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(127.531, 83.875, Math.toRadians(0))),
                            Math.toRadians(0));
                    transition(AutoState.BACK_TO_SHOOT);
                }
                break;

            case BACK_TO_SHOOT:
                if (inPhaseFor(800)) {
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

            case STACK2:
                if (atEnd()) {
                    follower.holdPoint(
                            new BezierPoint(new Pose(128.781, 59.594, Math.toRadians(0))),
                            Math.toRadians(0));
                    transition(AutoState.BACK_TO_SHOOT1);
                }
                break;

            case BACK_TO_SHOOT1:
                if (inPhaseFor(500)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(Shoot1, true);
                    transition(AutoState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                        follower.followPath(toGate1, true);
                        transition(AutoState.GATE1);
                    }
                }
                break;

            case GATE1:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE1), GATE_TAKE1.getHeading());
                    transition(AutoState.GATE1_HOLD);
                }
                break;

            case GATE1_HOLD:
                if (inPhaseFor(2000)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(shootFromGate1, true);
                    transition(AutoState.BACK_FROM_GATE1);
                }
                break;

            case BACK_FROM_GATE1:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    }
                    if (shotFired) {
                        transition(AutoState.SHOOTGATE1);
                    }
                }
                break;

            case SHOOTGATE1:
                if (robot.shootCommand.isFinished()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    follower.followPath(toGate2, true);
                    transition(AutoState.GATE2);
                }
                break;

            case GATE2:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE2), GATE_TAKE2.getHeading());
                    transition(AutoState.GATE2_HOLD);
                }
                break;

            case GATE2_HOLD:
                if (inPhaseFor(2000)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(shootFromGate2, true);
                    transition(AutoState.BACK_FROM_GATE2);
                }
                break;

            case BACK_FROM_GATE2:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    }
                    if (shotFired) {
                        transition(AutoState.SHOOTGATE2);
                    }
                }
                break;

            case SHOOTGATE2:
                if (robot.shootCommand.isFinished()) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    follower.followPath(toGate3, true);
                    transition(AutoState.GATE3);
                }
                break;

            case GATE3:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE3), GATE_TAKE3.getHeading());
                    transition(AutoState.GATE3_HOLD);
                }
                break;

            case GATE3_HOLD:
                if (inPhaseFor(2000)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(shootFromGate3, true);
                    transition(AutoState.BACK_FROM_GATE3);
                }
                break;

            case BACK_FROM_GATE3:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    }
                    if (shotFired) {
                        transition(AutoState.SHOOTGATE3);
                    }
                }
                break;

            case SHOOTGATE3:
                if (robot.shootCommand.isFinished()) {
                    transition(AutoState.DONE);
                }
                break;

            case DONE:
                break;
        }
    }
    private void buildPaths() {

        PreLoad = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.781, 121.688),
                        new Pose(99.984,   83.672)))
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();

        Stack1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(99.984,  83.672),
                        new Pose(127.531, 83.875)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(127.531, 83.875),
                        new Pose(98.625,  82.844)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Stack2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(98.625,  82.844),
                        new Pose(93.250,  55.359),
                        new Pose(128.781, 59.594)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(128.781, 59.594),
                        new Pose(96.922,  62.438),
                        new Pose(96.563,  80.594)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        toGate1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.563, 80.594),
                        GATE_TAKE1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();

        shootFromGate1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        GATE_TAKE1,
                        new Pose(96.438, 82.500)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        toGate2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.438, 82.500),
                        GATE_TAKE2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();

        shootFromGate2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        GATE_TAKE2,
                        new Pose(96.438, 82.500)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        toGate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.438, 82.500),
                        GATE_TAKE3))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();

        shootFromGate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        GATE_TAKE3,
                        new Pose(96.438, 82.500)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

    }

    // Utils
    private long phaseEntryTime = 0;

    private void transition(AutoState newState) {
        state          = newState;
        phaseEntryTime = System.currentTimeMillis();
        shotFired      = false;
        pathEndArmed   = false;
    }

    private boolean inPhaseFor(long ms) {
        return (System.currentTimeMillis() - phaseEntryTime) >= ms;
    }

    private ShooterSubsystem shooter() { return robot.shooter; }
}