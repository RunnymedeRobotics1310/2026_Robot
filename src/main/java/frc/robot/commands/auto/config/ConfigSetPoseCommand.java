package frc.robot.commands.auto.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigSetPoseCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final Pose2d targetPose;
    private Pose2d allianceTargetPose;

    public ConfigSetPoseCommand(
            SwerveSubsystem swerve,
            double xMetres,
            double yMetres,
            double headingDegrees) {
        this.swerve = swerve;
        this.targetPose = new Pose2d(xMetres, yMetres, Rotation2d.fromDegrees(headingDegrees));
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        allianceTargetPose = targetPose;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            allianceTargetPose = RunnymedeUtils.getRedAlliancePose(targetPose);
        }

        swerve.resetOdometry(allianceTargetPose);
        setFinishReason("Pose set to " + format(allianceTargetPose));
        logCommandStart("pose=" + format(allianceTargetPose));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}
