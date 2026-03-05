package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "set_pose", category = "drive",
    description = "Reset odometry to a specified pose")
public class SetPoseCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final Pose2d targetPose;
    private Pose2d allianceTargetPose;

    public SetPoseCommand(
            SwerveSubsystem swerve,
            @ConfigParam(value = "xMetres", unit = "m", min = 0, max = 16.54,
                description = "X position on field") double xMetres,
            @ConfigParam(value = "yMetres", unit = "m", min = 0, max = 8.07,
                description = "Y position on field") double yMetres,
            @ConfigParam(value = "headingDegrees", unit = "deg",
                description = "Robot heading") double headingDegrees) {
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
