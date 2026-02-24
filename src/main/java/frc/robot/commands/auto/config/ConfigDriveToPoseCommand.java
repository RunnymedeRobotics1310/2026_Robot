package frc.robot.commands.auto.config;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigDriveToPoseCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final Pose2d targetPose;
    private final double maxSpeedMPS;
    private final double positionToleranceMetres;
    private final double headingToleranceDegrees;
    private final double timeoutSeconds;

    private Pose2d allianceTargetPose;
    private double allianceHeadingDeg;

    public ConfigDriveToPoseCommand(
            SwerveSubsystem swerve,
            double xMetres,
            double yMetres,
            double headingDegrees,
            double maxSpeedMPS,
            double positionToleranceMetres,
            double headingToleranceDegrees,
            double timeoutSeconds) {
        this.swerve = swerve;
        this.targetPose = new Pose2d(xMetres, yMetres, Rotation2d.fromDegrees(headingDegrees));
        this.maxSpeedMPS = maxSpeedMPS;
        this.positionToleranceMetres = positionToleranceMetres;
        this.headingToleranceDegrees = headingToleranceDegrees;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            this.allianceTargetPose = RunnymedeUtils.getRedAlliancePose(targetPose);
        } else {
            this.allianceTargetPose = targetPose;
        }
        this.allianceHeadingDeg = SwerveUtils.normalizeDegrees(allianceTargetPose.getRotation().getDegrees());

        logCommandStart("pose=" + format(allianceTargetPose)
                + " maxSpd=" + maxSpeedMPS
                + " posTol=" + positionToleranceMetres
                + " hdgTol=" + headingToleranceDegrees
                + " timeout=" + timeoutSeconds);
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        double xDiff = allianceTargetPose.getX() - currentPose.getX();
        double yDiff = allianceTargetPose.getY() - currentPose.getY();
        Translation2d translationDiff = new Translation2d(xDiff, yDiff);
        Translation2d translationVelocity = swerve.computeVelocity(
                translationDiff, Math.min(maxSpeedMPS, Constants.Swerve.TRANSLATION_CONFIG.maxSpeedMPS()));

        double angleDiff = SwerveUtils.normalizeDegrees(allianceHeadingDeg - currentPose.getRotation().getDegrees());
        double maxOmega = Math.max((Math.toRadians(angleDiff) / Math.max(translationDiff.getNorm(), 0.05))
                * translationVelocity.getNorm(), 0.1);
        double omega = swerve.computeOmega(allianceHeadingDeg, maxOmega);

        swerve.driveFieldOriented(translationVelocity.getX(), translationVelocity.getY(), omega);
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(),
                allianceTargetPose.getTranslation(),
                positionToleranceMetres);
        boolean atHeading = SwerveUtils.isCloseEnough(
                swerve.getYaw(),
                allianceHeadingDeg,
                headingToleranceDegrees);

        if (atTarget && atHeading) {
            setFinishReason("Reached target pose");
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
