package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.tools.Hardware;
import org.firstinspires.ftc.teamcode.tools.PoseStorage;


@TeleOp(name = "TeleOp", group = "!")
public class TeleOperado extends OpMode {

    private Hardware         hw;
    private ShooterSubsystem shooter;
    private IntakeSubsystem  intake;
    private TurretSubsystem  turret;
    private Follower         follower;

    private DcMotorEx FL, FR, BL, BR;

    private enum Alliance { RED, BLUE }
    private Alliance alliance           = Alliance.RED;
    private boolean  lastBumperAlliance = false;

    private static final double RED_GOAL_X  = 142;
    private static final double RED_GOAL_Y  = 142;
    private static final double BLUE_GOAL_X = 144.0 - 142.0;
    private static final double BLUE_GOAL_Y = 142.0;

    private static final double TPS_CORNER = 1100.0;
    private static final double TPS_SAFE   = 1000.0;
    private static final double TPS_IDLE   =  800.0;
    private static final double TPS_FAR    = 1350.0;

    private enum RobotState { IDLE, COLLECTING, SHOOTING, OUTTAKE }
    private RobotState robotState = RobotState.IDLE;

    private boolean lastShootPressed = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setDirection(DcMotorEx.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hw = new Hardware(this, false);
        hw.init();

        shooter = new ShooterSubsystem(hw);
        intake  = new IntakeSubsystem(hw);
        turret  = new TurretSubsystem(hw);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);

        applyAlliance();

        // AIMBOT activo desde init — la torreta empieza a apuntar antes de start()
        turret.setMode(TurretSubsystem.TurretMode.AIMBOT);

        shooter.state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter.setTargetTPS(TPS_IDLE);

        telemetry.addData("Pinpoint Status", hw.pinpoint.getDeviceStatus());
        telemetry.addData("Pose heredada",   PoseStorage.currentPose.toString());
        telemetry.addData(">", "Listo — RB en init para cambiar alianza");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Actualiza follower para que getPose() sea fresco durante init_loop
        follower.update();

        boolean bumperNow = gamepad1.right_bumper;
        if (bumperNow && !lastBumperAlliance) {
            alliance = (alliance == Alliance.RED) ? Alliance.BLUE : Alliance.RED;
            applyAlliance();
        }
        lastBumperAlliance = bumperNow;

        // La torreta ya apunta durante init_loop — al darle start ya está convergida
        turret.update();

        telemetry.addData(">> ALIANZA <<",
                alliance == Alliance.RED ? "ROJO  (RB para cambiar)" : "AZUL  (RB para cambiar)");
        telemetry.addData("Turret atTarget", turret.atTarget());
        telemetry.addData("Turret Ticks",    turret.getCurrentTicks());
        telemetry.update();
    }

    @Override
    public void loop() {

        follower.update();

        // ── Drivetrain ────────────────────────────────────────────────────────
        double y     = -gamepad1.left_stick_y;
        double x     =  (gamepad1.left_stick_x * 1.1);
        double rx    =  gamepad1.right_stick_x;
        double denom =  Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        setDrive(y, x, rx, denom);

        // ── Flywheel ──────────────────────────────────────────────────────────
        if      (gamepad1.x) shooter.setTargetTPS(TPS_CORNER);
        else if (gamepad1.b) shooter.setTargetTPS(TPS_SAFE);
        else if (gamepad1.a) shooter.setTargetTPS(TPS_IDLE);
        else if (gamepad1.y) shooter.setTargetTPS(TPS_FAR);

        // ── Turret ────────────────────────────────────────────────────────────
        // AIMBOT continuo — el subsistema calcula azimuth internamente cada loop.
        // gamepad2.a centra la torreta en home (HOLD a 0 ticks).
        // gamepad2 stick permite override manual puntual; al soltar regresa a AIMBOT.
        double turretStick = gamepad2.right_stick_x;
        if (Math.abs(turretStick) > 0.05) {
            turret.setMode(TurretSubsystem.TurretMode.MANUAL);
            turret.setManualPower(turretStick * 0.4);
        } else if (gamepad2.a) {
            // Centro manual explícito: HOLD en 0 ticks
            turret.setMode(TurretSubsystem.TurretMode.HOLD);
            turret.setTargetTicks(0);
        } else {
            // Caso nominal: AIMBOT continuo
            // setMode() es no-op si ya está en AIMBOT, sin costo
            turret.setMode(TurretSubsystem.TurretMode.AIMBOT);
        }

        // ── Input ─────────────────────────────────────────────────────────────
        boolean outtakePressed = gamepad1.left_trigger  > 0.05;
        boolean collectPressed = gamepad1.right_trigger > 0.05;
        boolean shootNow       = gamepad1.right_bumper;
        boolean shootRising    = shootNow && !lastShootPressed;
        lastShootPressed       = shootNow;

        // ── Robot FSM ─────────────────────────────────────────────────────────
        switch (robotState) {
            case IDLE:
                if (outtakePressed) {
                    robotState = RobotState.OUTTAKE;
                } else if (collectPressed) {
                    robotState = RobotState.COLLECTING;
                } else if (shootRising) {
                    // Gate: flywheel a velocidad Y torreta en azimuth correcto
                    if (shooter.isAtSpeed() && turret.atTarget()) {
                        if (shooter.triggerShoot()) robotState = RobotState.SHOOTING;
                    }
                }
                break;

            case COLLECTING:
                if (outtakePressed) {
                    robotState = RobotState.OUTTAKE;
                } else if (shootRising) {
                    // Mismo gate que IDLE
                    if (shooter.isAtSpeed() && turret.atTarget()) {
                        if (shooter.triggerShoot()) robotState = RobotState.SHOOTING;
                    }
                } else if (!collectPressed) {
                    robotState = RobotState.IDLE;
                }
                break;

            case SHOOTING:
                if (shooter.isShootDone()) {
                    shooter.resetShootFSM();
                    robotState = collectPressed ? RobotState.COLLECTING : RobotState.IDLE;
                }
                break;

            case OUTTAKE:
                if (!outtakePressed) robotState = RobotState.IDLE;
                break;
        }

        // ── Bridge RobotState → IntakeSubsystem ───────────────────────────────
        switch (robotState) {
            case COLLECTING: intake.setState(IntakeSubsystem.IntakeState.COLLECTING); break;
            case OUTTAKE:    intake.setState(IntakeSubsystem.IntakeState.OUTTAKE);    break;
            case SHOOTING:   intake.setState(IntakeSubsystem.IntakeState.SHOOTING);   break;
            default:         intake.setState(IntakeSubsystem.IntakeState.IDLE);       break;
        }

        // ── Subsystem updates ─────────────────────────────────────────────────
        turret.update();
        shooter.update();
        intake.update();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Alianza",         alliance);
        telemetry.addData("Estado",          robotState);
        telemetry.addData("FW Target TPS",   shooter.getTargetTPS());
        telemetry.addData("FW Actual TPS",   shooter.getCurrentTPS());
        telemetry.addData("FW Ready",        shooter.isAtSpeed());
        telemetry.addData("Shoot State",     shooter.isBursting() ? "BURSTING"
                : shooter.isShootDone() ? "DONE" : "IDLE");
        telemetry.addData("Turret Mode",     turret.getMode());
        telemetry.addData("Turret Ticks",    turret.getCurrentTicks());
        telemetry.addData("Turret Target",   turret.getTargetTicks());
        telemetry.addData("Turret atTarget", turret.atTarget());
        telemetry.addData("Turret HOME",     turret.getMode() == TurretSubsystem.TurretMode.HOLD
                && turret.getTargetTicks() == 0
                ? (turret.atTarget() ? "CENTRADO ✓" : "moviendo...") : "");
        telemetry.addData("Intake State",    intake.getState());
        telemetry.addData("Pose",            follower.getPose());
        telemetry.update();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private void applyAlliance() {
        double goalX = (alliance == Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = (alliance == Alliance.RED) ? RED_GOAL_Y : BLUE_GOAL_Y;
        turret.setGoal(goalX, goalY);
    }

    private void setDrive(double y, double x, double rx, double denom) {
        FL.setPower((y + x + rx) / denom);
        BL.setPower((y - x + rx) / denom);
        FR.setPower((y - x - rx) / denom);
        BR.setPower((y + x - rx) / denom);
    }

    /**
     * Normaliza un ángulo al rango [-π, π].
     * Evita el bug de Java donde % retiene el signo del dividendo,
     * lo que produce saltos al lado opuesto cuando rad ≈ ±π.
     */
    private double normalizeAngle(double rad) {
        rad = rad % (2.0 * Math.PI);
        if (rad >  Math.PI) rad -= 2.0 * Math.PI;
        if (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}