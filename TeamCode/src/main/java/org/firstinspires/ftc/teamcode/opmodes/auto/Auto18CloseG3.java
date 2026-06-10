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
@Autonomous(name = "18 Close G3 [RED]", group = "!")
public class Auto18CloseG3 extends OpMode {


    private static final Pose START_POSE  = new Pose(128.500, 111.000, Math.toRadians(-90));
    private static final Pose SHOOT_POSE  = new Pose(90.844,   76.922, Math.toRadians(0));
    private static final Pose GATE_TAKE1 = new Pose(144.5,  50, Math.toRadians(33));
    private static final Pose GATE_TAKE2 = new Pose(144.5,  50, Math.toRadians(33));
    private static final Pose GATE_TAKE3 = new Pose(144.5,  50, Math.toRadians(33));

    private static final long DRIVE_SETTLE_MS = 40;

    private static final long STACK_INTAKE_MS = 400;
    private static final long GATE_INTAKE_MS  = 1500;

    private Follower  follower;
    private AutoRobot robot;

    private PathChain PreLoad;
    private PathChain toStack1;
    private PathChain Shoot1;
    private PathChain toStack2;
    private PathChain Shoot2;
    private PathChain toGate1;
    private PathChain shootFromGate;
    private PathChain toGate2;
    private PathChain ShootFromGate2;
    private PathChain toGate3;
    private PathChain shootFromGate3;

    private AutoState state = AutoState.PRELOAD;

    private enum AutoState {
        PRELOAD,        SHOOT_PRELOAD,
        STACK1,         BACK_TO_SHOOT,  SHOOT,
        STACK2,         BACK_TO_SHOOT1, SHOOT1,
        GATE1,          GATE1_HOLD,     BACK_FROM_GATE,  SHOOTGATE1,
        GATE2,          GATE2_HOLD,     BACK_FROM_GATE2, SHOOTGATE2,
        GATE3,          GATE3_HOLD,     BACK_FROM_GATE3, SHOOTGATE3,
        DONE
    }
    private final ElapsedTime matchTimer = new ElapsedTime();

    // shotFired:    garantiza exactamente un shootCommand.start() por fase de disparo.
    // pathEndArmed: true solo después de DRIVE_SETTLE_MS en el estado actual.
    private boolean shotFired    = false;
    private boolean pathEndArmed = false;

    // ─── INIT ─────────────────────────────────────────────────────────────────
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

        telemetry.addLine("Auto18CloseG3 [RED] — Ready");
        telemetry.update();
    }

    // ─── START ────────────────────────────────────────────────────────────────
    @Override
    public void start() {
        matchTimer.reset();
        robot.start();
        shooter().state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter().setTargetVelocity(AutoRobot.SHOOT_FAR_TPS);
        follower.followPath(PreLoad, true);
        state = AutoState.PRELOAD;
    }

    // ─── LOOP ─────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        follower.update();
        robot.update();

        // Armar el flag de end solo después de DRIVE_SETTLE_MS en el estado actual.
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

    // ─── FSM ──────────────────────────────────────────────────────────────────
    private void runFSM() {
        switch (state) {

            // ── PRELOAD ──────────────────────────────────────────────────────
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
                    // Intake ya activo via callback en toStack1 a t=0.5
                    follower.followPath(toStack1, true);
                    transition(AutoState.STACK1);
                }
                break;

            // ── STACK 1 ──────────────────────────────────────────────────────
            // FIX: sin holdPoint — el robot queda en brake mode en la pose final.
            // El intake se activó vía parametricCallback a t=0.5 en toStack1.
            case STACK1:
                if (atEnd()) {
                    transition(AutoState.BACK_TO_SHOOT);
                }
                break;

            case BACK_TO_SHOOT:
                // STACK_INTAKE_MS garantiza contacto mecánico mínimo con el disco.
                if (inPhaseFor(STACK_INTAKE_MS)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(Shoot1, true);
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
                        // Intake via callback en toStack2 a t=0.5
                        follower.followPath(toStack2, true);
                        transition(AutoState.STACK2);
                    }
                }
                break;

            // ── STACK 2 ──────────────────────────────────────────────────────
            // FIX: sin holdPoint — mismo patrón que STACK1.
            case STACK2:
                if (atEnd()) {
                    transition(AutoState.BACK_TO_SHOOT1);
                }
                break;

            case BACK_TO_SHOOT1:
                if (inPhaseFor(STACK_INTAKE_MS)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(Shoot2, true);
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

            // ── CICLO GATE 1 ─────────────────────────────────────────────────
            case GATE1:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE1), GATE_TAKE1.getHeading());
                    transition(AutoState.GATE1_HOLD);
                }
                break;

            case GATE1_HOLD:
                // Salida temprana si el intake reporta disco; fallback a GATE_INTAKE_MS.
                // Sustituir robot.intake.hasDisc() si el subsistema expone el sensor.
                if (inPhaseFor(GATE_INTAKE_MS)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(shootFromGate, true);
                    transition(AutoState.BACK_FROM_GATE);
                }
                break;

            case BACK_FROM_GATE:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
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

            // ── CICLO GATE 2 ─────────────────────────────────────────────────
            case GATE2:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE2), GATE_TAKE2.getHeading());
                    transition(AutoState.GATE2_HOLD);
                }
                break;

            case GATE2_HOLD:
                if (inPhaseFor(GATE_INTAKE_MS)) {
                    robot.intake.setState(IntakeSubsystem.IntakeState.IDLE);
                    follower.followPath(ShootFromGate2, true);
                    transition(AutoState.BACK_FROM_GATE2);
                }
                break;

            case BACK_FROM_GATE2:
                if (atEnd()) {
                    if (!shotFired) {
                        robot.shootCommand = robot.buildShootCommand(AutoRobot.SHOOT_FAR_TPS);
                        robot.shootCommand.start();
                        shotFired = true;
                    } else if (robot.shootCommand.isFinished()) {
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

            // ── CICLO GATE 3 ─────────────────────────────────────────────────
            case GATE3:
                if (atEnd()) {
                    follower.holdPoint(new BezierPoint(GATE_TAKE3), GATE_TAKE3.getHeading());
                    transition(AutoState.GATE3_HOLD);
                }
                break;

            case GATE3_HOLD:
                if (inPhaseFor(GATE_INTAKE_MS)) {
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
                    } else if (robot.shootCommand.isFinished()) {
                        transition(AutoState.SHOOTGATE3);
                    }
                }
                break;

            case SHOOTGATE3:
                if (robot.shootCommand.isFinished()) {
                    transition(AutoState.DONE);
                }
                break;
        }
    }

    // ─── BUILD PATHS ──────────────────────────────────────────────────────────
    private void buildPaths() {

        // ── PRELOAD ───────────────────────────────────────────────────────────
        PreLoad = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, new Pose(111.000, 94.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        // ── STACK 1 ───────────────────────────────────────────────────────────
        // FIX: callback activa el intake a t=0.5 — elimina la latencia de la transición.
        toStack1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(111.000, 94.000),
                        new Pose(118.000, 94.000),
                        new Pose(122.233, 84)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addParametricCallback(0.5, () ->
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING))
                .build();

        // FIX: callback apaga el intake a t=0.8 — no espera el atEnd() de SHOOT.
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.233, 84),
                        new Pose(110.000, 94.000)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addParametricCallback(0.8, () ->
                        robot.intake.setState(IntakeSubsystem.IntakeState.IDLE))
                .build();

        // ── STACK 2 ───────────────────────────────────────────────────────────
        toStack2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(110.000, 94.000),
                        new Pose(119.000, 82.750),
                        new Pose(122.233, 58.5)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addParametricCallback(0.5, () ->
                        robot.intake.setState(IntakeSubsystem.IntakeState.COLLECTING))
                .build();

        // FIX (bug original): Shoot2 tenía el path de Shoot1 copy-pasteado.
        // Ahora parte desde la pose real del stack 2 hacia SHOOT_POSE.
        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.233, 58.5),
                        SHOOT_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addParametricCallback(0.8, () ->
                        robot.intake.setState(IntakeSubsystem.IntakeState.IDLE))
                // Frena un poco más tarde pero con mayor fuerza — neto: llega rápido y clava en SHOOT_POSE.
                .build();

        // ── GATE CYCLES ───────────────────────────────────────────────────────
        // toGate: sin deceleration — el robot no necesita precisión al entrar al gate,
        // el holdPoint se encarga de posicionar exacto. Se ahorra ~100ms por ciclo.
        toGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        SHOOT_POSE,
                        new Pose(114,41),
                        GATE_TAKE1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        // shootFromGate: frena más tarde pero con fuerza — llega rápido y para preciso en SHOOT_POSE.
        shootFromGate = follower.pathBuilder()
                .addPath(new BezierLine(GATE_TAKE1, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        toGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        SHOOT_POSE,
                        new Pose(114,41),
                        GATE_TAKE2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        ShootFromGate2 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_TAKE2, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        toGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        SHOOT_POSE,
                        new Pose(114,41),
                        GATE_TAKE3))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        shootFromGate3 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_TAKE3, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
    }

    // ─── UTILS ────────────────────────────────────────────────────────────────

    // atEnd(): true solo si el settle guard ya se armó Y Pedro reporta fin de path.
// atEnd() más robusto — usa !isBusy() como condición primaria
    private boolean atEnd() {
        return pathEndArmed && !follower.isBusy();
    }

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