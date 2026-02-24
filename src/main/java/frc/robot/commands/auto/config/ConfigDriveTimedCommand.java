package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigDriveTimedCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final double direction;
    private final double speedMPS;
    private final double durationSeconds;
    private final double heading;

    private double allianceDirection;
    private double allianceHeading;

    public ConfigDriveTimedCommand(
            SwerveSubsystem swerve,
            double direction,
            double speedMPS,
            double durationSeconds,
            double heading) {
        this.swerve = swerve;
        this.direction = direction;
        this.speedMPS = speedMPS;
        this.durationSeconds = durationSeconds;
        this.heading = heading;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double headingOffset = 0;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            headingOffset = 180;
        }
        allianceDirection = direction + headingOffset;
        allianceHeading = heading + headingOffset;

        logCommandStart("dir=" + allianceDirection + " spd=" + speedMPS
                + " dur=" + durationSeconds + " hdg=" + allianceHeading);
    }

    @Override
    public void execute() {
        double dirRad = Math.toRadians(allianceDirection);
        double vx = speedMPS * Math.cos(dirRad);
        double vy = speedMPS * Math.sin(dirRad);
        double omega = swerve.computeOmega(allianceHeading);
        swerve.driveFieldOriented(vx, vy, omega);
    }

    @Override
    public boolean isFinished() {
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
