package frc.robot.commands.auto.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigDriveDistanceCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final double direction;
    private final double speedMPS;
    private final double distanceMetres;
    private final double heading;
    private final double timeoutSeconds;

    private double allianceDirection;
    private double allianceHeading;
    private Pose2d startPose;

    public ConfigDriveDistanceCommand(
            SwerveSubsystem swerve,
            double direction,
            double speedMPS,
            double distanceMetres,
            double heading,
            double timeoutSeconds) {
        this.swerve = swerve;
        this.direction = direction;
        this.speedMPS = speedMPS;
        this.distanceMetres = distanceMetres;
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
        allianceDirection = direction + headingOffset;
        allianceHeading = heading + headingOffset;
        startPose = swerve.getPose();

        logCommandStart("dir=" + allianceDirection + " spd=" + speedMPS
                + " dist=" + distanceMetres + " hdg=" + allianceHeading);
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
        double dx = swerve.getPose().getX() - startPose.getX();
        double dy = swerve.getPose().getY() - startPose.getY();
        double traveled = Math.sqrt(dx * dx + dy * dy);

        if (traveled >= distanceMetres) {
            setFinishReason("Distance reached: " + format(traveled) + "m");
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
