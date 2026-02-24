package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ConfigShooterCommand extends LoggingCommand {

    private final ShooterSubsystem shooter;
    private final AutoStep.ShooterAction action;
    private final double rpm;
    private final double hoodPosition;
    private final double kickerSpeed;
    private final double kickerDelaySeconds;
    private final double durationSeconds;

    private final Timer timer = new Timer();
    private boolean kickerStarted = false;

    public ConfigShooterCommand(
            ShooterSubsystem shooter,
            AutoStep.ShooterAction action,
            double rpm,
            double hoodPosition,
            double kickerSpeed,
            double kickerDelaySeconds,
            double durationSeconds) {
        this.shooter = shooter;
        this.action = action;
        this.rpm = rpm;
        this.hoodPosition = hoodPosition;
        this.kickerSpeed = kickerSpeed;
        this.kickerDelaySeconds = kickerDelaySeconds;
        this.durationSeconds = durationSeconds;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        logCommandStart("action=" + action + " rpm=" + rpm + " hood=" + hoodPosition);
        timer.reset();
        timer.start();
        kickerStarted = false;

        if (action == null || action == AutoStep.ShooterAction.off) {
            shooter.stop();
        } else {
            shooter.setShooterVelocity(rpm);
            shooter.setHood(hoodPosition);
        }
    }

    @Override
    public void execute() {
        if (action == null || action == AutoStep.ShooterAction.off) {
            return;
        }

        shooter.setShooterVelocity(rpm);

        if (!kickerStarted && kickerDelaySeconds > 0 && timer.hasElapsed(kickerDelaySeconds)) {
            shooter.setKickerSpeed(kickerSpeed);
            kickerStarted = true;
        } else if (kickerDelaySeconds <= 0 && !kickerStarted) {
            shooter.setKickerSpeed(kickerSpeed);
            kickerStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (action == null) {
            setFinishReason("Invalid shooter action");
            return true;
        }
        if (action == AutoStep.ShooterAction.off) {
            setFinishReason("Shooter off");
            return true;
        }
        if (action == AutoStep.ShooterAction.on_with_duration && timer.hasElapsed(durationSeconds)) {
            setFinishReason("Duration elapsed: " + durationSeconds + "s");
            return true;
        }
        // "on" action runs until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (action != AutoStep.ShooterAction.on) {
            shooter.stop();
        }
        logCommandEnd(interrupted);
    }
}
