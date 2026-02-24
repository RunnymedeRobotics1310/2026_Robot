package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigDriveVelocityCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final AutoStep.VelocityFrame frame;
    private final double vxMPS;
    private final double vyMPS;
    private final double headingDegrees;
    private final double durationSeconds;

    private double allianceHeading;
    private double allianceVx;
    private double allianceVy;

    public ConfigDriveVelocityCommand(
            SwerveSubsystem swerve,
            AutoStep.VelocityFrame frame,
            double vxMPS,
            double vyMPS,
            double headingDegrees,
            double durationSeconds) {
        this.swerve = swerve;
        this.frame = frame;
        this.vxMPS = vxMPS;
        this.vyMPS = vyMPS;
        this.headingDegrees = headingDegrees;
        this.durationSeconds = durationSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double headingOffset = 0;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            headingOffset = 180;
        }
        allianceHeading = headingDegrees + headingOffset;

        if (frame == AutoStep.VelocityFrame.field && headingOffset != 0) {
            // Red alliance field transform rotates by 180 degrees.
            allianceVx = -vxMPS;
            allianceVy = -vyMPS;
        } else {
            allianceVx = vxMPS;
            allianceVy = vyMPS;
        }

        logCommandStart("frame=" + frame
                + " vx=" + allianceVx
                + " vy=" + allianceVy
                + " hdg=" + allianceHeading
                + " dur=" + durationSeconds);
    }

    @Override
    public void execute() {
        double omega = swerve.computeOmega(allianceHeading);
        if (frame == AutoStep.VelocityFrame.robot) {
            swerve.driveRobotOriented(allianceVx, allianceVy, omega);
        } else {
            swerve.driveFieldOriented(allianceVx, allianceVy, omega);
        }
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
