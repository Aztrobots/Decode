package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Variante de AutoRobot optimizada para ciclos gate rápidos (Explotado21).
 * AutoCommand está embebido — no depende de AutoRobot en absoluto.
 *
 * Diferencias respecto a AutoRobot:
 *  - AT_SPEED_TIMEOUT_MS: 800 → 500 ms
 *  - buildShootCommand: Phase 1 espera isAtSpeed() AND turret.atTarget()
 */
public class AutoRobotFast {

    public final Hardware         hw;
    public final IntakeSubsystem  intake;
    public final ShooterSubsystem shooter;
    public final TurretSubsystem  turret;

    public final AutoCommand intakeCommand;
    public AutoCommand       shootCommand;

    public static double SHOOT_FAR_TPS  = 1080;
    public static double SHOOT_NEAR_TPS = 1050;

    public static long AT_SPEED_TIMEOUT_MS = 500;

    public AutoRobotFast(OpMode opMode) {
        hw = new Hardware(opMode, false, false);
        hw.init();
        hw.turret.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.turret.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake  = new IntakeSubsystem(hw);
        shooter = new ShooterSubsystem(hw);
        turret  = new TurretSubsystem(hw);

        intakeCommand = new AutoCommand(
                new AutoCommand.Phase(() -> {
                    intake.resetDetection();
                    intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
                    shooter.resetShootFSM();
                }, 200, () -> false),

                new AutoCommand.Phase(null,
                        1000, () -> intake.intakeFull()),

                new AutoCommand.Phase(() -> {
                    intake.setState(IntakeSubsystem.IntakeState.IDLE);
                }, 100, () -> false)
        );

        shootCommand = buildShootCommand(SHOOT_FAR_TPS);
    }

    /**
     * Phase 0 — Setea velocidad y resetea FSM. (10 ms — lógicamente instantáneo)
     * Phase 1 — Espera flywheel isAtSpeed() AND turret.atTarget().
     *           Timeout AT_SPEED_TIMEOUT_MS como fallback.
     * Phase 2 — Trigger. (10 ms)
     * Phase 3 — Espera gate open. Exit: isBursting().
     * Phase 4 — Espera burst completo. Exit: isShootDone().
     * Phase 5 — Limpia FSM. (10 ms — lógicamente instantáneo)
     */
    public AutoCommand buildShootCommand(double velocityTPS) {
        return new AutoCommand(
                new AutoCommand.Phase(() -> {
                    shooter.setTargetVelocity(velocityTPS);
                    shooter.resetShootFSM();
                }, 10, () -> false),

                new AutoCommand.Phase(
                        null,
                        AT_SPEED_TIMEOUT_MS,
                        () -> shooter.isAtSpeed() && turret.atTarget()),

                new AutoCommand.Phase(() -> {
                    shooter.triggerShootAuto();
                }, 10, () -> false),

                new AutoCommand.Phase(
                        null,
                        (long)(ShooterSubsystem.GATE_OPEN_DELAY_S * 1000) + 10,
                        () -> shooter.isBursting()),

                new AutoCommand.Phase(
                        null,
                        (long)(ShooterSubsystem.BURST_DURATION * 1000) + 100,
                        () -> shooter.isShootDone()),

                new AutoCommand.Phase(() -> {
                    shooter.resetShootFSM();
                }, 10, () -> false)
        );
    }

    public void init() {
        initPositions();
    }

    public void start() {
        shooter.state = ShooterSubsystem.ShooterState.SHOOTER_ON;
        shooter.setTargetVelocity(SHOOT_FAR_TPS);
    }

    public void update() {
        intake.update();
        shooter.update();
        turret.update();
        intakeCommand.update();
        shootCommand.update();

        if (shooter.isBursting()) {
            intake.setState(IntakeSubsystem.IntakeState.COLLECTING);
        }
    }

    public void initPositions() {
        shooter.resetShootFSM();
        shooter.setTargetVelocity(0);
        shooter.state = ShooterSubsystem.ShooterState.SHOOTER_OFF;
        turret.setMode(TurretSubsystem.TurretMode.HOLD);
        intake.setState(IntakeSubsystem.IntakeState.IDLE);
    }

    public void setAzimuthThetaVelocity(double azimuthRad, double thetaRad, double velocityTPS) {
        turret.setTargetRadians(azimuthRad);
        shooter.setTargetVelocity(velocityTPS);
    }

    // ─── AutoCommand ─────────────────────────────────────────────────────────

    public static class AutoCommand {
        public interface ExitCondition { boolean check(); }

        public static class Phase {
            final Runnable      onEnter;
            final long          maxTimeMs;
            final ExitCondition exit;
            public Phase(Runnable onEnter, long maxTimeMs, ExitCondition exit) {
                this.onEnter   = onEnter;
                this.maxTimeMs = maxTimeMs;
                this.exit      = exit;
            }
        }

        private final Phase[]     phases;
        private int               currentPhase = -1;
        private boolean           finished     = false;
        private final ElapsedTime phaseTimer   = new ElapsedTime();

        public AutoCommand(Phase... phases) { this.phases = phases; }

        public void start() {
            currentPhase = 0;
            finished     = false;
            phaseTimer.reset();
            if (phases[0].onEnter != null) phases[0].onEnter.run();
        }

        public void update() {
            if (finished || currentPhase < 0) return;
            Phase p         = phases[currentPhase];
            boolean expired = phaseTimer.milliseconds() >= p.maxTimeMs;
            boolean condMet = p.exit.check();
            if (expired || condMet) {
                currentPhase++;
                if (currentPhase >= phases.length) { finished = true; return; }
                phaseTimer.reset();
                if (phases[currentPhase].onEnter != null) phases[currentPhase].onEnter.run();
            }
        }

        public boolean isFinished() { return finished; }
        public boolean isRunning()  { return currentPhase >= 0 && !finished; }
    }
}