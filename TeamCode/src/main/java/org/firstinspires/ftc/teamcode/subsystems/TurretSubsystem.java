package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.tools.Hardware;

/**
 * TurretSubsystem — refactored.
 *
 * Correcciones vs. versión original:
 *
 *  1. pinpoint.update() REMOVIDO de aquí — vive en AutoRobot.update() para
 *     garantizar exactamente una llamada por loop. Llamarlo dos veces por
 *     iteración causa lecturas de posición inconsistentes.
 *
 *  2. API unificada rad/ticks:
 *     - setTargetTicks(double ticks)   → clampea y setea directamente (igual que antes)
 *     - setTargetRadians(double rad)   → convierte rad→ticks internamente y clampea
 *     AutoRobot.setAzimuthThetaVelocity() debe usar setTargetRadians() para evitar
 *     la multiplicación por TICKS_PER_RAD fuera del subsistema.
 *
 *  3. updateAimbot() preservado intacto — lee la Pose2D del Pinpoint que ya fue
 *     actualizado por AutoRobot.update() en el mismo loop tick.
 *
 *  4. Clamp de ticks documentado: ±262/+312 ticks ≈ ±2.2 rad / +2.6 rad.
 *     El rango físico de la torreta limita el aimbot cuando el robot está muy
 *     cerca del goal en ángulo extremo — comportamiento esperado.
 *
 *  5. getTargetRadians() añadido para telemetría.
 */
@Config
public class TurretSubsystem {

    // ─── CONSTANTES MECÁNICAS ─────────────────────────────────────────────────
    public static double TICKS_PER_REV = 384.54 * 1.95; // 749.85 t/rev
    public static double TICKS_PER_RAD = TICKS_PER_REV / (2 * Math.PI); // ~119.33

    // Límites físicos del recorrido de la torreta
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

    // ─── OBJETIVO EN EL CAMPO (configurable desde Dashboard) ─────────────────
    public static double GOAL_X            = 142.0;  // in, coordenadas Pedro — RED
    public static double GOAL_Y            = 142.0;
    public static double TURRET_OFFSET_RAD = 0.0;    // offset de montaje físico

    // ─── ESTADO ──────────────────────────────────────────────────────────────
    public enum TurretMode { AIMBOT, MANUAL, HOLD }
    private TurretMode mode = TurretMode.MANUAL;

    private double targetTicks = 0;
    private double lastError   = 0;
    private double manualPower = 0;

    private final Hardware hw;

    // ─── CONSTRUCTOR ─────────────────────────────────────────────────────────
    public TurretSubsystem(Hardware hw) {
        this.hw          = hw;
        this.targetTicks = clampTicks(hw.turret.getCurrentPosition());
    }

    // ─── API PÚBLICA ──────────────────────────────────────────────────────────

    public void setMode(TurretMode newMode) {
        if (this.mode != newMode) {
            lastError = 0;
            // Al entrar en HOLD o AIMBOT, congela el target en la posición actual
            // para evitar un salto brusco en el primer tick
            if (newMode == TurretMode.HOLD || newMode == TurretMode.AIMBOT) {
                targetTicks = clampTicks(hw.turret.getCurrentPosition());
            }
        }
        this.mode = newMode;
    }

    public TurretMode getMode() { return mode; }

    public void setManualPower(double power) { this.manualPower = power; }

    /**
     * Setea target directamente en ticks (ya calculados externamente).
     * Clampea al rango físico.
     */
    public void setTargetTicks(double ticks) {
        targetTicks = clampTicks(ticks);
    }

    /**
     * Setea target en radianes — convierte internamente a ticks.
     * Usar este método desde AutoRobot.setAzimuthThetaVelocity() para mantener
     * la conversión encapsulada en el subsistema.
     *
     * @param radians ángulo relativo al robot (0 = frente, positivo = izquierda)
     */
    public void setTargetRadians(double radians) {
        targetTicks = clampTicks(radians * TICKS_PER_RAD);
    }

    public double getCurrentTicks()   { return hw.turret.getCurrentPosition(); }
    public double getTargetTicks()    { return targetTicks; }
    public double getTargetRadians()  { return targetTicks / TICKS_PER_RAD; }
    public double getCurrentRadians() { return getCurrentTicks() / TICKS_PER_RAD; }

    public void setGoal(double x, double y) { GOAL_X = x; GOAL_Y = y; }

    public boolean atTarget() {
        return Math.abs(getCurrentTicks() - targetTicks) < AT_TARGET_TOL;
    }

    // ─── UPDATE ───────────────────────────────────────────────────────────────

    /**
     * Llamar una vez por loop().
     * PRECONDICIÓN: hw.pinpoint.update() ya fue llamado en este mismo tick
     * (AutoRobot.update() lo garantiza).
     */
    public void update() {
        switch (mode) {
            case AIMBOT: updateAimbot(); break;
            case MANUAL: updateManual(); break;
            case HOLD:   runPD(targetTicks); break;
        }
    }

    // ─── MODOS PRIVADOS ───────────────────────────────────────────────────────

    private void updateAimbot() {
        // Lee pose del Pinpoint — ya actualizado por AutoRobot.update()
        Pose2D pose     = hw.pinpoint.getPosition();
        double robotX   = pose.getX(DistanceUnit.INCH);
        double robotY   = pose.getY(DistanceUnit.INCH);
        double robotHdg = hw.pinpoint.getHeading(AngleUnit.RADIANS);

        double angleToGoal   = Math.atan2(GOAL_Y - robotY, GOAL_X - robotX);
        double relativeAngle = normalizeAngle(angleToGoal - robotHdg - TURRET_OFFSET_RAD);

        // Conversión rad → ticks encapsulada aquí
        targetTicks = clampTicks(relativeAngle * TICKS_PER_RAD);
        runPD(targetTicks);
    }

    private void updateManual() {
        double currentTicks = getCurrentTicks();
        if (Math.abs(manualPower) > 0.05) {
            // Límites físicos en modo manual
            if ((manualPower > 0 && currentTicks >= MAX_TICKS) ||
                    (manualPower < 0 && currentTicks <= MIN_TICKS)) {
                hw.turret.setPower(0);
            } else {
                hw.turret.setPower(manualPower * MAX_POWER);
            }
            // Actualiza target para que HOLD tome la posición actual al soltar
            targetTicks = clampTicks(currentTicks);
            lastError   = 0;
        } else {
            // Joystick suelto → mantener posición
            runPD(targetTicks);
        }
    }

    // ─── CONTROLADOR DUAL-PD + kS ─────────────────────────────────────────────

    private void runPD(double target) {
        double current    = getCurrentTicks();
        double error      = target - current;
        double derivative = error - lastError;
        lastError = error;

        double kP, kD, kS;
        if (Math.abs(error) > PIDF_SWITCH_TICKS) {
            kP = KP_FAR;  kD = KD_FAR;  kS = KS_FAR;
        } else {
            kP = KP_NEAR; kD = KD_NEAR; kS = KS_NEAR;
        }

        double power = kP * error + kD * derivative;

        // kS: feedforward estático para vencer fricción (solo fuera de tolerancia)
        if (Math.abs(error) >= AT_TARGET_TOL) {
            power += kS * Math.signum(error);
        }

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        hw.turret.setPower(power);
    }

    // ─── UTILIDADES ───────────────────────────────────────────────────────────

    private double clampTicks(double ticks) {
        return Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
    }

    private double normalizeAngle(double rad) {
        while (rad >  Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}