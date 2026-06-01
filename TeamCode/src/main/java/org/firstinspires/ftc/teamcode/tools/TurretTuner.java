package org.firstinspires.ftc.teamcode.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp(name = "Turret PIDF Tuner", group = "Tuning")
public class TurretTuner extends OpMode {

    private DcMotorEx turretMotor;


    private static final double TICKS_PER_REV = 384.54 * 1.95; // 749.85
    private static final double TICKS_PER_RAD = TICKS_PER_REV / (2 * Math.PI); // ~119.33



    public static double PIDF_SWITCH_TICKS = 40.0;


    public static double kP_FAR  = 0.0129;
    public static double kD_FAR  = 0.0002;
    public static double kS_FAR  = 0.2;

    public static double kP_NEAR = 0.0218;
    public static double kD_NEAR = 0.0002;
    public static double kS_NEAR = 0.2;


    public static double TARGET_DEG = 0.0;


    public static double MAX_POWER = 0.45;


    public static double AT_TARGET_TOL = 10.0;

    private double lastError     = 0.0;
    private double lastTimestamp = 0.0;

    private MultipleTelemetry tel;



    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lastTimestamp = getRuntime();
        tel.update();
    }

    @Override
    public void loop() {

        double currentTicks = turretMotor.getCurrentPosition();
        double targetTicks  = Math.toRadians(TARGET_DEG) * TICKS_PER_RAD;


        double now   = getRuntime();
        double dt    = now - lastTimestamp;
        double error = targetTicks - currentTicks;

        double derivative = (dt > 1e-6) ? (error - lastError) / dt : 0.0;

        lastError     = error;
        lastTimestamp = now;

        // Selección dual-PIDF (Baron-style) ─────────────────────────────
        double kP, kD, kS;
        String activeController;

        if (Math.abs(error) > PIDF_SWITCH_TICKS) {
            kP = kP_FAR;
            kD = kD_FAR;
            kS = kS_FAR;
            activeController = "FAR";
        } else {
            kP = kP_NEAR;
            kD = kD_NEAR;
            kS = kS_NEAR;
            activeController = "NEAR";
        }


        double power = kP * error
                + kD * derivative
                + kS * Math.signum(error); // static friction compensation (decode-style)

        // Clamp + deadband de kS cuando el error es cero ────────────────
        // Evita que kS mantenga el motor energizado cuando ya está en target
        if (Math.abs(error) < AT_TARGET_TOL) {
            power = kP * error + kD * derivative; // sin kS dentro de tolerancia
        }

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turretMotor.setPower(power);


        boolean atTarget = Math.abs(error) < AT_TARGET_TOL;

        tel.addData("── Controller ──",    activeController);
        tel.addData("Target (deg)",        TARGET_DEG);
        tel.addData("Target (ticks)",      String.format("%.1f", targetTicks));
        tel.addData("Current (ticks)",     currentTicks);
        tel.addData("Error (ticks)",       String.format("%.1f", error));
        tel.addData("Error (deg)",         String.format("%.2f", error / TICKS_PER_RAD * 180.0 / Math.PI));
        tel.addData("Derivative",          String.format("%.4f", derivative));
        tel.addData("Output power",        String.format("%.4f", power));
        tel.addData("At Target?",          atTarget ? "✓ YES" : "✗ NO");
        tel.addData("── Constants ──",     "");
        tel.addData("kP / kD / kS (active)", String.format("%.4f / %.5f / %.4f", kP, kD, kS));
        tel.addData("PIDF_SWITCH_TICKS",   PIDF_SWITCH_TICKS);
        tel.addData("── Mechanical ──",    "");
        tel.addData("TICKS_PER_REV",       String.format("%.2f", TICKS_PER_REV));
        tel.addData("TICKS_PER_RAD",       String.format("%.3f", TICKS_PER_RAD));
        tel.update();
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
    }
}