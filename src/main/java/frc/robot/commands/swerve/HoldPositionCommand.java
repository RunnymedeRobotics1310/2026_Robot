package frc.robot.commands.swerve;

import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "hold", category = "drive",
    description = "Hold position for a duration")
public class HoldPositionCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final double durationSeconds;

    public HoldPositionCommand(
            SwerveSubsystem swerve,
            @ConfigParam(value = "durationSeconds", unit = "s", min = 0, max = 15,
                description = "Duration to hold (0 = indefinite)") double durationSeconds) {
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
