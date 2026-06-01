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

/**
 * TeleOperado — drivetrain manual + AIMBOT del turret con tracking de Pedro.
 *
 * Diseño de odometría:
 *   Hardware(this, false) → ownsPinpoint=true, resetPinpoint=false.
 *     - Configura encoderResolution y encoderDirections (FORWARD/REVERSED) sin resetear.
 *     - La pose acumulada del Pinpoint se mantiene.
 *   createFollower() → Pedro crea PinpointLocalizer con la misma config → tracking live.
 *     - NO se llama setStartingPose() → Pedro arranca desde su origen (0,0,0) y trackea
 *       en vivo. Mismo comportamiento que TeleOperadoHonduras (validado).
 *   follower.update() en loop → solo updatePose() cuando no hay path activo
 *     (Follower.update() retorna en "if (currentPath == null) return") → no interfiere
 *     con el drivetrain manual.
 *
 * Nota: NO llamar follower.startTeleopDrive() → eso activa manualDrive=true y Pedro
 *   intentaría controlar los motores. Usamos follower solo para odometría aquí.
 *
 * Botones:
 *   GP1 left_trigger  → OUTTAKE
 *   GP1 right_trigger → COLLECTING
 *   GP1 right_bumper  → SHOOT (flanco de subida)
 *   GP1 x             → TPS_LOW  (1080)
 *   GP1 b             → TPS_HIGH (1400)
 *   GP1 a             → TPS_IDLE (800)
 *   GP2 right_stick_x → Turret manual
 *   GP2 y             → Turret vuelve a AIMBOT
 *   GP1 right_bumper (init_loop) → toggle alianza
 */
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


    private static final double TPS_LOW  = 1080.0;
    private static final double TPS_HIGH = 1400.0;
    private static final double TPS_IDLE =  800.0;


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

        applyAlliance();
        turret.setMode(TurretSubsystem.TurretMode.AIMBOT);

        shooter.state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter.setTargetTPS(TPS_IDLE);

        telemetry.addData("Pinpoint Status", hw.pinpoint.getDeviceStatus());
        telemetry.addData(">", "Listo — RB en init para cambiar alianza");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        boolean bumperNow = gamepad1.right_bumper;
        if (bumperNow && !lastBumperAlliance) {
            alliance = (alliance == Alliance.RED) ? Alliance.BLUE : Alliance.RED;
            applyAlliance();
        }
        lastBumperAlliance = bumperNow;

        telemetry.addData(">> ALIANZA <<",
                alliance == Alliance.RED ? "ROJO  (RB para cambiar)" : "AZUL  (RB para cambiar)");
        telemetry.update();
    }


    @Override
    public void loop() {

        follower.update();


        double y     = -gamepad1.left_stick_y*0.75;
        double x     =  (gamepad1.left_stick_x * 1.1)*0.75;
        double rx    =  gamepad1.right_stick_x*0.75;
        double denom =  Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        setDrive(y, x, rx, denom);


        if      (gamepad1.x) shooter.setTargetTPS(TPS_LOW);
        else if (gamepad1.b) shooter.setTargetTPS(TPS_HIGH);
        else if (gamepad1.a) shooter.setTargetTPS(TPS_IDLE);


        double turretStick = gamepad2.right_stick_x;
        if (Math.abs(turretStick) > 0.05) {
            turret.setMode(TurretSubsystem.TurretMode.MANUAL);
            turret.setManualPower(turretStick * 0.4);
        } else if (gamepad2.y) {
            turret.setMode(TurretSubsystem.TurretMode.AIMBOT);
        }


        boolean outtakePressed = gamepad1.left_trigger  > 0.05;
        boolean collectPressed = gamepad1.right_trigger > 0.05;
        boolean shootNow       = gamepad1.right_bumper;
        boolean shootRising    = shootNow && !lastShootPressed;
        lastShootPressed       = shootNow;

        switch (robotState) {
            case IDLE:
                if (outtakePressed) {
                    robotState = RobotState.OUTTAKE;
                } else if (collectPressed) {
                    robotState = RobotState.COLLECTING;
                } else if (shootRising) {
                    shooter.triggerShoot();
                    robotState = RobotState.SHOOTING;
                }
                break;

            case COLLECTING:
                if (outtakePressed) {
                    robotState = RobotState.OUTTAKE;
                } else if (!collectPressed) {
                    robotState = RobotState.IDLE;
                } else if (shootRising) {
                    shooter.triggerShoot();
                    robotState = RobotState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (outtakePressed) {
                    shooter.resetShootFSM();
                    robotState = RobotState.OUTTAKE;
                } else if (shooter.isShootDone()) {
                    shooter.resetShootFSM();
                    robotState = collectPressed ? RobotState.COLLECTING : RobotState.IDLE;
                }
                break;

            case OUTTAKE:
                if (!outtakePressed) {
                    robotState = RobotState.IDLE;
                }
                break;
        }


        switch (robotState) {
            case IDLE:
                intake.setState(IntakeSubsystem.IntakeState.IDLE);
                break;
            case COLLECTING:
                intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                break;
            case SHOOTING:
                intake.setState(shooter.isBursting()
                        ? IntakeSubsystem.IntakeState.SHOOTING
                        : IntakeSubsystem.IntakeState.IDLE);
                break;
            case OUTTAKE:
                intake.setState(IntakeSubsystem.IntakeState.OUTTAKE);
                break;
        }


        turret.update();
        shooter.update();
        intake.update();


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
        telemetry.addData("Pose",            follower.getPose());
        telemetry.update();
    }


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
}