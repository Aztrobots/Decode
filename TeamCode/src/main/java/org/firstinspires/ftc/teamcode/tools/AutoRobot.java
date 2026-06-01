package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class AutoRobot {

    public final Hardware         hw;
    public final IntakeSubsystem  intake;
    public final ShooterSubsystem shooter;
    public final TurretSubsystem  turret;

    public final AutoCommand intakeCommand;
    public AutoCommand       shootCommand;

    public static double SHOOT_FAR_TPS  = 1150;
    public static double SHOOT_NEAR_TPS = 1000;

    public AutoRobot(OpMode opMode) {
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
                    shooter.closeLatch();
                }, 200, () -> false),

                new AutoCommand.Phase(null,
                        1700, () -> intake.intakeFull()),

                new AutoCommand.Phase(() -> {
                    intake.setState(IntakeSubsystem.IntakeState.IDLE);
                }, 100, () -> false)
        );

        shootCommand = buildShootCommand(SHOOT_FAR_TPS);
    }

    // shoot command factory
    public AutoCommand buildShootCommand(double velocityTPS) {
        return new AutoCommand(
                // Phase 0: ajusta velocidad y activa el burst
                new AutoCommand.Phase(() -> {
                    shooter.setTargetVelocity(velocityTPS);
                    shooter.resetBurst();
                    shooter.shootBurstForced();
                }, 50, () -> false),

                // Phase 1: espera a que el gate abra
                new AutoCommand.Phase(
                        null,
                        (long)(ShooterSubsystem.GATE_OPEN_DELAY_S * 1000) + 50,
                        () -> shooter.isBursting()),

                // Phase 2: espera a que el burst complete
                new AutoCommand.Phase(
                        null,
                        (long)(ShooterSubsystem.BURST_DURATION * 1000) + 100,
                        () -> shooter.isBurstDone()),

                // Phase 3: limpia el burst
                new AutoCommand.Phase(() -> {
                    shooter.resetBurst();
                }, 50, () -> false)
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

        if (shooter.isShooting()) {
            intake.setState(shooter.isBursting()
                    ? IntakeSubsystem.IntakeState.COLLECTING
                    : IntakeSubsystem.IntakeState.IDLE);
        }
    }

    public void initPositions() {
        shooter.closeLatch();
        shooter.setTargetVelocity(0);
        shooter.state = ShooterSubsystem.ShooterState.SHOOTER_OFF;
        turret.setMode(TurretSubsystem.TurretMode.HOLD);
        intake.setState(IntakeSubsystem.IntakeState.IDLE);
    }

    public void setAzimuthThetaVelocity(double azimuthRad, double thetaRad, double velocityTPS) {
        turret.setTargetRadians(azimuthRad);
        shooter.setTargetVelocity(velocityTPS);
    }



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