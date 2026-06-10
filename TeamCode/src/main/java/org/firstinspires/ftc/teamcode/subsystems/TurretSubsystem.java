package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Hardware;

@Config
public class TurretSubsystem {

    public static double TICKS_PER_REV = 384.54 * 1.95; // 749.85 t/rev

    private static double ticksPerRad() { return TICKS_PER_REV / (2.0 * Math.PI); }

    public static double MAX_TICKS =  312.0;
    public static double MIN_TICKS = -262.0;

    // PIDF
    // KP_FAR más agresivo para convergencia rápida en recorridos largos.
    // KP_NEAR conservador para evitar oscilación en zona fina de afinado.
    public static double KP_FAR  = 0.055;
    public static double KD_FAR  = 0.0005;
    public static double KS_FAR  = 0.22;

    public static double KP_NEAR = 0.0218;
    public static double KD_NEAR = 0.0002;
    public static double KS_NEAR = 0.2;

    public static double PIDF_SWITCH_TICKS = 40.0;

    // MAX_POWER elevado a 0.85 para dar headroom real en recorridos largos.
    // La zona NEAR lo reduce internamente al 65% para no oscilar en la zona fina.
    public static double MAX_POWER         = 0.85;
    public static double AT_TARGET_TOL     = 10.0;

    // Goal
    public static double GOAL_X            = 144.0;
    public static double GOAL_Y            = 144.0;
    public static double TURRET_OFFSET_RAD = 0.0;

    // Estados
    public enum TurretMode { AIMBOT, MANUAL, HOLD }
    private TurretMode mode = TurretMode.MANUAL;

    private double targetTicks  = 0;
    private double lastPosition = 0;
    private double manualPower  = 0;

    // ─── SOTM FEEDFORWARD ─────────────────────────────────────────────────────
    private double feedforward = 0.0;

    private final ElapsedTime pdTimer    = new ElapsedTime();
    private boolean           pdFirstRun = true;

    private final Hardware hw;

    // Constructor
    public TurretSubsystem(Hardware hw) {
        this.hw           = hw;
        this.targetTicks  = clampTicks(hw.turret.getCurrentPosition());
        this.lastPosition = this.targetTicks;
    }

    // ── API ───────────────────────────────────────────────────────────────────

    public void setMode(TurretMode newMode) {
        if (this.mode == newMode) return;
        if (newMode == TurretMode.HOLD || newMode == TurretMode.AIMBOT) {
            targetTicks = clampTicks(hw.turret.getCurrentPosition());
        }
        lastPosition = hw.turret.getCurrentPosition();
        pdFirstRun   = true;
        feedforward  = 0.0;
        this.mode    = newMode;
    }

    public TurretMode getMode() { return mode; }

    public void setManualPower(double power) { this.manualPower = power; }

    /** Feedforward de SOTM (motor-power). Refrescar cada loop mientras se hace SOTM. */
    public void setFeedforward(double ff) { this.feedforward = ff; }

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

    public boolean isAzimuthReachable(double radians) {
        double ticks = radians * ticksPerRad();
        return ticks <= MAX_TICKS && ticks >= MIN_TICKS;
    }

    // ── UPDATE ────────────────────────────────────────────────────────────────

    public void update() {
        switch (mode) {
            case AIMBOT: updateAimbot(); break;
            case MANUAL: updateManual(); break;
            case HOLD:   runPD(targetTicks); break;
        }
    }

    private void updateAimbot() {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose =
                hw.pinpoint.getPosition();
        double robotX   = pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double robotY   = pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double robotHdg = hw.pinpoint.getHeading(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

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

    // ── PD ────────────────────────────────────────────────────────────────────

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

        // D sobre posición — evita derivative kick cuando aimbot actualiza target cada loop
        double dPosition = (dt > 0.001) ? (current - lastPosition) / dt : 0.0;
        lastPosition = current;

        double kP, kD, kS, effectiveMaxPower;

        if (Math.abs(error) > PIDF_SWITCH_TICKS) {
            // Zona FAR: ganancias agresivas + MAX_POWER completo para convergencia rápida
            kP = KP_FAR;
            kD = KD_FAR;
            kS = KS_FAR;
            effectiveMaxPower = MAX_POWER;
        } else {
            // Zona NEAR: ganancias suaves + power reducido para evitar oscilación fina
            kP = KP_NEAR;
            kD = KD_NEAR;
            kS = KS_NEAR;
            effectiveMaxPower = MAX_POWER * 0.65;
        }

        double power = kP * error - kD * dPosition;

        if (Math.abs(error) >= AT_TARGET_TOL) {
            power += kS * Math.signum(error);
        }

        // FF de SOTM: solo en HOLD
        if (mode == TurretMode.HOLD) {
            power += feedforward;
        }

        hw.turret.setPower(Math.max(-effectiveMaxPower, Math.min(effectiveMaxPower, power)));
    }

    // ── Utils ─────────────────────────────────────────────────────────────────

    private double clampTicks(double ticks) {
        return Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
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