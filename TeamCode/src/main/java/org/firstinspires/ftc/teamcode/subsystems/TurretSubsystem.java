package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Hardware;

@Config
public class TurretSubsystem {

    // ─── CONSTANTES MECÁNICAS ─────────────────────────────────────────────────
    public static double TICKS_PER_REV = 384.54 * 1.95; // 749.85 t/rev

    // Derivado en runtime para evitar desincronización si Dashboard modifica TICKS_PER_REV
    private static double ticksPerRad() { return TICKS_PER_REV / (2.0 * Math.PI); }

    public static double MAX_TICKS =  312.0;
    public static double MIN_TICKS = -262.0;

    // ─── GANANCIAS ────────────────────────────────────────────────────────────
    public static double KP_FAR  = 0.03;
    public static double KD_FAR  = 0.0004;
    public static double KS_FAR  = 0.2;

    public static double KP_NEAR = 0.0218;
    public static double KD_NEAR = 0.0002;
    public static double KS_NEAR = 0.2;

    public static double PIDF_SWITCH_TICKS = 40.0;
    public static double MAX_POWER         = 0.65;
    public static double AT_TARGET_TOL     = 10.0;

    // ─── OBJETIVO EN EL CAMPO ─────────────────────────────────────────────────
    public static double GOAL_X            = 142.0;
    public static double GOAL_Y            = 142.0;
    public static double TURRET_OFFSET_RAD = 0.0;

    // ─── ESTADO ───────────────────────────────────────────────────────────────
    public enum TurretMode { AIMBOT, MANUAL, HOLD }
    private TurretMode mode = TurretMode.MANUAL;

    private double targetTicks  = 0;
    private double lastPosition = 0;
    private double manualPower  = 0;

    private final ElapsedTime pdTimer  = new ElapsedTime();
    private boolean           pdFirstRun = true;

    private final Hardware hw;

    // ─── CONSTRUCTOR ──────────────────────────────────────────────────────────
    public TurretSubsystem(Hardware hw) {
        this.hw           = hw;
        this.targetTicks  = clampTicks(hw.turret.getCurrentPosition());
        this.lastPosition = this.targetTicks;
    }

    // ─── API PÚBLICA ──────────────────────────────────────────────────────────

    public void setMode(TurretMode newMode) {
        if (this.mode == newMode) return;
        if (newMode == TurretMode.HOLD || newMode == TurretMode.AIMBOT) {
            targetTicks = clampTicks(hw.turret.getCurrentPosition());
        }
        lastPosition = hw.turret.getCurrentPosition();
        pdFirstRun   = true;
        this.mode    = newMode;
    }

    public TurretMode getMode() { return mode; }

    public void setManualPower(double power) { this.manualPower = power; }

    public void setTargetTicks(double ticks) {
        targetTicks = clampTicks(ticks);
    }

    public void setTargetRadians(double radians) {
        targetTicks = clampTicks(radians * ticksPerRad());
    }

    public void setGoal(double x, double y) { GOAL_X = x; GOAL_Y = y; }

    public double getCurrentTicks()   { return hw.turret.getCurrentPosition(); }
    public double getTargetTicks()    { return targetTicks; }
    public double getTargetRadians()  { return targetTicks / ticksPerRad(); }
    public double getCurrentRadians() { return getCurrentTicks() / ticksPerRad(); }

    public boolean atTarget() {
        return Math.abs(getCurrentTicks() - targetTicks) < AT_TARGET_TOL;
    }

    // ─── UPDATE ───────────────────────────────────────────────────────────────

    public void update() {
        switch (mode) {
            case AIMBOT: updateAimbot(); break;
            case MANUAL: updateManual(); break;
            case HOLD:   runPD(targetTicks); break;
        }
    }

    // ─── MODOS PRIVADOS ───────────────────────────────────────────────────────

    private void updateAimbot() {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose =
                hw.pinpoint.getPosition();
        double robotX   = pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double robotY   = pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double robotHdg = hw.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

        double angleToGoal   = Math.atan2(GOAL_Y - robotY, GOAL_X - robotX);
        double relativeAngle = normalizeAngle(angleToGoal - robotHdg - TURRET_OFFSET_RAD);

        targetTicks = clampTicks(relativeAngle * ticksPerRad());
        runPD(targetTicks);
    }

    private void updateManual() {
        double currentTicks = getCurrentTicks();
        if (Math.abs(manualPower) > 0.05) {
            if ((manualPower > 0 && currentTicks >= MAX_TICKS) ||
                    (manualPower < 0 && currentTicks <= MIN_TICKS)) {
                hw.turret.setPower(0);
            } else {
                hw.turret.setPower(manualPower * MAX_POWER);
            }
            targetTicks  = clampTicks(currentTicks);
            lastPosition = currentTicks;
            pdFirstRun   = true;
        } else {
            runPD(targetTicks);
        }
    }

    // ─── CONTROLADOR DUAL-PD + kS ────────────────────────────────────────────

    private void runPD(double target) {
        double current = getCurrentTicks();
        double error   = target - current;

        if (pdFirstRun) {
            pdTimer.reset();
            lastPosition = current;
            pdFirstRun   = false;
        }

        double dt = pdTimer.seconds();
        pdTimer.reset();

        // D sobre posición — evita derivative kick cuando el aimbot actualiza target cada loop
        double dPosition = (dt > 0.001) ? (current - lastPosition) / dt : 0.0;
        lastPosition = current;

        double kP, kD, kS;
        if (Math.abs(error) > PIDF_SWITCH_TICKS) {
            kP = KP_FAR;  kD = KD_FAR;  kS = KS_FAR;
        } else {
            kP = KP_NEAR; kD = KD_NEAR; kS = KS_NEAR;
        }

        double power = kP * error - kD * dPosition;

        if (Math.abs(error) >= AT_TARGET_TOL) {
            power += kS * Math.signum(error);
        }

        hw.turret.setPower(Math.max(-MAX_POWER, Math.min(MAX_POWER, power)));
    }

    // ─── UTILIDADES ───────────────────────────────────────────────────────────

    private double clampTicks(double ticks) {
        return Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
    }

    // O(1) en lugar de while-loop
    private double normalizeAngle(double rad) {
        return ((rad + Math.PI) % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI) - Math.PI;
    }
}