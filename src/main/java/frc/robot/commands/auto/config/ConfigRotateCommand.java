package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigRotateCommand extends LoggingCommand {

    private static final double TOLERANCE_DEGREES = 2.0;

    private final SwerveSubsystem swerve;
    private final double heading;
    private final double timeoutSeconds;

    private double allianceHeading;

    public ConfigRotateCommand(SwerveSubsystem swerve, double heading, double timeoutSeconds) {
        this.swerve = swerve;
        this.heading = heading;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double headingOffset = 0;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            headingOffset = 180;
        }
        allianceHeading = heading + headingOffset;

        logCommandStart("hdg=" + allianceHeading + " timeout=" + timeoutSeconds);
    }

    @Override
    public void execute() {
        double omega = swerve.computeOmega(allianceHeading);
        swerve.driveFieldOriented(0, 0, omega);
    }

    @Override
    public boolean isFinished() {
        double currentHeading = swerve.getYaw();
        double error = Math.abs(currentHeading - allianceHeading);
        // Normalize to [-180, 180]
        while (error > 180) {
            error -= 360;
        }
        error = Math.abs(error);

        if (error <= TOLERANCE_DEGREES) {
            setFinishReason("Heading reached: " + format(currentHeading) + " deg");
            return true;
        }
        if (timeoutSeconds > 0 && hasElapsed(timeoutSeconds)) {
            setFinishReason("Timeout after " + timeoutSeconds + "s");
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
