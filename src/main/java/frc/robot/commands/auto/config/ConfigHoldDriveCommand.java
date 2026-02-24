package frc.robot.commands.auto.config;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigHoldDriveCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final double durationSeconds;

    public ConfigHoldDriveCommand(SwerveSubsystem swerve, double durationSeconds) {
        this.swerve = swerve;
        this.durationSeconds = durationSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        logCommandStart("duration=" + durationSeconds);
    }

    @Override
    public void execute() {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        if (durationSeconds <= 0) {
            return false;
        }
        if (hasElapsed(durationSeconds)) {
            setFinishReason("Duration elapsed: " + durationSeconds + "s");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        logCommandEnd(interrupted);
    }
}
