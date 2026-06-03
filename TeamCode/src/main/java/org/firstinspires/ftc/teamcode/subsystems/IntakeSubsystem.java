package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Hardware;

@Config
public class IntakeSubsystem {

    // ─── CONFIGURABLES ────────────────────────────────────────────────────────
    public static double INTAKE_POWER      =  1.0;
    public static double INTAKE_SLOW_POWER =  0.8;
    public static double OUTTAKE_POWER     = -1.0;

    public static double FULL_TIMEOUT_MS = 1200.0;

    public enum IntakeState {
        IDLE,
        COLLECTING,
        COLLECTING_SLOW,
        SHOOTING,
        OUTTAKE
    }

    private IntakeState state = IntakeState.IDLE;

    private final ElapsedTime collectTimer = new ElapsedTime();
    private boolean collecting   = false;
    private boolean detectedFull = false;

    private double cachedIntakePw = Double.NaN;

    private final Hardware hw;

    public IntakeSubsystem(Hardware hw) { this.hw = hw; }

    // ─── API ──────────────────────────────────────────────────────────────────

    public void setState(IntakeState newState) {
        if (this.state != newState) {
            if (newState == IntakeState.COLLECTING) {
                if (!collecting) {
                    collecting = true;
                    collectTimer.reset();
                }
            } else {
                collecting = false;
            }
        }
        this.state = newState;
    }

    public IntakeState getState() { return state; }

    public boolean intakeFull() { return detectedFull; }

    public void resetDetection() {
        detectedFull = false;
        collecting   = false;
    }

    // ─── UPDATE ───────────────────────────────────────────────────────────────

    public void update() {
        if (collecting && collectTimer.milliseconds() >= FULL_TIMEOUT_MS) {
            detectedFull = true;
        }

        switch (state) {
            case IDLE:
                setIntake(0);
                break;
            case COLLECTING:
                setIntake(INTAKE_POWER);
                break;
            case COLLECTING_SLOW:
            case SHOOTING:
                setIntake(INTAKE_SLOW_POWER);
                break;
            case OUTTAKE:
                setIntake(OUTTAKE_POWER);
                break;
        }
    }

    // ─── INTERNAL ─────────────────────────────────────────────────────────────

    private void setIntake(double power) {
        // NaN-safe: igual que flywheel, primer ciclo siempre pasa
        if (Double.isNaN(cachedIntakePw) || cachedIntakePw != power) {
            hw.intake.setPower(power);
            cachedIntakePw = power;
        }
    }
}