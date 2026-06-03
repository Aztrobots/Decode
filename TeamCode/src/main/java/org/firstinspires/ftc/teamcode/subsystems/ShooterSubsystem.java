package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Hardware;

@Config
public class ShooterSubsystem {

    // PIDF
    public static double F         = 0.0004545454;
    public static double P         = 0.005;
    public static double I         = 0.0;
    public static double D         = 0.0;
    public static double TOLERANCE = 30.0;
    public static double MAX_TPS   = 2200.0;

    public static double GATE_OPEN_DELAY_S   = 0.15;
    public static double BURST_DURATION      = 0.5;
    public static double BURST_FORCE_TIMEOUT = 1.0;

    public enum ShooterState { SHOOTER_ON, SHOOTER_OFF }
    public ShooterState state = ShooterState.SHOOTER_OFF;

    public enum ShootState { IDLE, OPENING, BURSTING, DONE }
    private ShootState shootState = ShootState.IDLE;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean           forcedBurst = false;

    private double        targetTPS   = 0;
    private double        lastError   = 0;
    private double        integralSum = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean       pidFirstRun = true;

    // caché
    private double cachedGatePos    = Double.NaN;
    private double cachedFlywheelPw = Double.NaN;
    private static final double POWER_CACHE_EPSILON = 1e-6;

    private final Hardware hw;

    public ShooterSubsystem(Hardware hw) { this.hw = hw; }

    // Getters

    public void setTargetVelocity(double tps) { setTargetTPS(tps); }

    public void setTargetTPS(double tps) {
        targetTPS = Math.min(Math.abs(tps), MAX_TPS);
        if (tps <= 0) {
            integralSum = 0;
            lastError   = 0;
            pidFirstRun = true;
        }
    }

    public double getTargetTPS() { return targetTPS; }

    // Usa Math.min: el PIDF reacciona al motor más rezagado (worst-case)
    public double getCurrentTPS() {
        return Math.min(
                Math.abs(hw.flyWheel.getVelocity()),
                Math.abs(hw.fwl.getVelocity())
        );
    }

    public boolean atTarget()  { return isAtSpeed(); }

    public boolean isAtSpeed() {
        return targetTPS > 0 && Math.abs(targetTPS - getCurrentTPS()) < TOLERANCE;
    }


    public void openLatch()  { setGate(Hardware.GATE_OPEN);  }
    public void closeLatch() { setGate(Hardware.GATE_BLOCK); }

    //Burst API

    public boolean shootBurst() {
        if (shootState == ShootState.DONE) return true;
        if (shootState == ShootState.IDLE) {
            if (!isAtSpeed()) return false;
            startBurst(false);
        }
        return false;
    }

    public boolean shootBurstForced() {
        if (shootState == ShootState.DONE) return true;
        if (shootState == ShootState.IDLE) startBurst(true);
        return false;
    }

    public void resetBurst() {
        shootState  = ShootState.IDLE;
        forcedBurst = false;
        closeLatch();
    }

    public void triggerShoot() {
        if (shootState == ShootState.IDLE) startBurst(!isAtSpeed());
    }

    // Queries

    public boolean isIntakeActive() { return shootState == ShootState.BURSTING; }
    public boolean isShooting()     { return shootState == ShootState.OPENING || shootState == ShootState.BURSTING; }
    public boolean isBursting()     { return shootState == ShootState.BURSTING; }
    public boolean isShootDone()    { return shootState == ShootState.DONE; }
    public boolean isBurstDone()    { return shootState == ShootState.DONE; }
    public void    resetShootFSM()  { resetBurst(); }
    public ShootState getShootState() { return shootState; }



    public void update() {
        updateFlywheel();
        updateBurstFSM();
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
                double burstTimeout = forcedBurst ? BURST_FORCE_TIMEOUT : BURST_DURATION;
                if (shootTimer.seconds() >= burstTimeout) {
                    closeLatch();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    // INTERNALS

    private void startBurst(boolean forced) {
        forcedBurst = forced;
        openLatch();
        shootTimer.reset();
        shootState = ShootState.OPENING;
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

        double error = targetTPS - getCurrentTPS();
        double dt    = pidTimer.seconds();
        pidTimer.reset();

        double derivative = (dt > 0.001) ? (error - lastError) / dt : 0;

        // Anti-windup: no acumula integral cuando el output está saturado
        double rawPower  = (F * targetTPS) + (P * error) + (I * integralSum) + (D * derivative);
        boolean saturated = rawPower >= 1.0 || rawPower <= 0.0;
        if (Math.abs(error) < TOLERANCE * 2 && !saturated) {
            integralSum += error * dt;
        }

        lastError = error;

        setFlywheelPower(Math.max(0, Math.min(1, rawPower)));
    }

    private void setGate(double pos) {
        if (cachedGatePos != pos) {
            hw.leftGate.setPosition(pos);
            cachedGatePos = pos;
        }
    }

    private void setFlywheelPower(double power) {
        if (Double.isNaN(cachedFlywheelPw) || Math.abs(cachedFlywheelPw - power) > POWER_CACHE_EPSILON) {
            hw.flyWheel.setPower(power);
            hw.fwl.setPower(power);
            cachedFlywheelPw = power;
        }
    }
}