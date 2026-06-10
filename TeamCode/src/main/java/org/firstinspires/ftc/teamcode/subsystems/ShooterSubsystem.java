package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Hardware;

@Config
public class ShooterSubsystem {

    // ─── GANANCIAS PIDF ───────────────────────────────────────────────────────
    public static double F         = 0.00054945;
    public static double P         = 0.0045;     // era 0.0015 — recuperación post-burst más agresiva
    public static double I         = 0.0;
    public static double D         = 0.0;
    public static double TOLERANCE = 85.0;       // era 40 — isAtSpeed() dispara antes sin penalizar precisión
    public static double MAX_TPS   = 1820.0;

    public static double GATE_OPEN_DELAY_S = 0;
    public static double BURST_DURATION    = 0.55;

    // ─── ESTADOS ──────────────────────────────────────────────────────────────
    public enum ShooterState  { SHOOTER_ON, SHOOTER_OFF }
    public ShooterState state = ShooterState.SHOOTER_OFF;

    public enum ShootState { IDLE, OPENING, BURSTING, DONE }
    private ShootState shootState = ShootState.IDLE;

    // ─── PIDF STATE ───────────────────────────────────────────────────────────
    private double            targetTPS   = 0;
    private double            lastError   = 0;
    private double            integralSum = 0;
    private final ElapsedTime pidTimer    = new ElapsedTime();
    private boolean           pidFirstRun = true;

    // ─── BURST TIMER ─────────────────────────────────────────────────────────
    private final ElapsedTime shootTimer = new ElapsedTime();

    // ─── CACHE ───────────────────────────────────────────────────────────────
    private double cachedGatePos    = Double.NaN;
    private double cachedFlywheelPw = Double.NaN;

    private final Hardware hw;

    public ShooterSubsystem(Hardware hw) { this.hw = hw; }

    // ─── API PÚBLICA ──────────────────────────────────────────────────────────

    public void setTargetTPS(double tps) {
        targetTPS = Math.min(Math.abs(tps), MAX_TPS);
        if (tps <= 0) { integralSum = 0; lastError = 0; }
    }

    public void setRawPower(double power) {
        state = ShooterState.SHOOTER_OFF;  // congela PIDF
        hw.flyWheel.setPower(power);
        hw.fwl.setPower(power);
        cachedFlywheelPw = power;
    }

    /** Alias para compatibilidad con llamadas existentes. */
    public void setTargetVelocity(double tps) { setTargetTPS(tps); }

    public double getTargetTPS()  { return targetTPS; }

    public double getCurrentTPS() {
        return (Math.abs(hw.flyWheel.getVelocity()) + Math.abs(hw.fwl.getVelocity())) / 2.0;
    }

    public boolean isAtSpeed() {
        return targetTPS > 0 && Math.abs(targetTPS - getCurrentTPS()) < TOLERANCE;
    }

    /**
     * Dispara SOLO si el flywheel está dentro de TOLERANCE del targetTPS.
     * El gate NUNCA abre si no está a velocidad. Solo para TeleOp.
     *
     * @return true si el burst fue iniciado, false si fue bloqueado por velocidad.
     */
    public boolean triggerShoot() {
        if (shootState != ShootState.IDLE) return false;
        if (!isAtSpeed())                  return false;   // HARD BLOCK
        startBurst();
        return true;
    }

    /**
     * Inicia burst incondicionalmente. Solo para uso en Auto.
     * TeleOp SIEMPRE usa triggerShoot().
     */
    public void triggerShootAuto() {
        if (shootState != ShootState.IDLE) return;
        startBurst();
    }

    public void resetShootFSM() {
        shootState = ShootState.IDLE;
        closeLatch();
    }

    // ─── QUERIES DE ESTADO ────────────────────────────────────────────────────

    public boolean isBursting()  { return shootState == ShootState.BURSTING; }
    public boolean isShootDone() { return shootState == ShootState.DONE;     }
    public ShootState getShootState() { return shootState; }

    // ─── UPDATE LOOP ──────────────────────────────────────────────────────────

    public void update() {
        updateFlywheel();
        updateBurstFSM();
    }

    // ─── INTERNOS ─────────────────────────────────────────────────────────────

    private void startBurst() {
        openLatch();
        shootTimer.reset();
        shootState = ShootState.OPENING;
    }

    private void updateBurstFSM() {
        switch (shootState) {
            case IDLE:
                closeLatch();
                break;

            case OPENING:
                if (shootTimer.seconds() >= GATE_OPEN_DELAY_S) {
                    shootTimer.reset();
                    shootState = ShootState.BURSTING;
                }
                break;

            case BURSTING:
                if (shootTimer.seconds() >= BURST_DURATION) {
                    closeLatch();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void updateFlywheel() {
        if (state == ShooterState.SHOOTER_OFF || targetTPS <= 0) {
            setFlywheelPower(0);
            pidFirstRun = true;
            lastError   = 0;
            integralSum = 0;
            return;
        }

        if (pidFirstRun) {
            pidTimer.reset();
            pidFirstRun = false;
        }

        double error      = targetTPS - getCurrentTPS();
        double dt         = pidTimer.seconds();
        pidTimer.reset();

        double derivative = (dt > 0.001) ? (error - lastError) / dt : 0;

        // Anti-windup: clamp a >= 0 porque el flywheel no puede frenarse activamente.
        // Evita que el integral acumule negativos durante la rampa de bajada (far→near TPS).
        if (Math.abs(error) < TOLERANCE * 2) {
            integralSum = Math.max(0, integralSum + error * dt);
        } else {
            integralSum = 0;
        }

        lastError = error;

        double power = (F * targetTPS) + (P * error) + (I * integralSum) + (D * derivative);
        setFlywheelPower(Math.max(0, Math.min(1, power)));
    }

    private void openLatch()  { setGate(Hardware.GATE_OPEN);  }
    private void closeLatch() { setGate(Hardware.GATE_BLOCK); }

    private void setGate(double pos) {
        if (cachedGatePos != pos) { hw.leftGate.setPosition(pos); cachedGatePos = pos; }
    }

    private void setFlywheelPower(double power) {
        if (cachedFlywheelPw != power) {
            hw.flyWheel.setPower(power);
            hw.fwl.setPower(power);
            cachedFlywheelPw = power;
        }
    }
}