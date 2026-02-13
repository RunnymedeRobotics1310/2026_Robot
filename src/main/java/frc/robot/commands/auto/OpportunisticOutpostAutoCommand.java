package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class OpportunisticOutpostAutoCommand extends SequentialCommandGroup {

    public OpportunisticOutpostAutoCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision) {
    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(new DriveToFieldLocationCommand(swerve,
            new Pose2d(0.8, 0.6, Rotation2d.fromDegrees(0))));
    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, -0.3, 0, 0)
            .withTimeout(2));
    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 1, 2, 0)
        .withTimeout(2));

    addCommands(new FaceHubCommand(swerve));
    addCommands(new NullDriveCommand(swerve).withTimeout(3));

    addCommands(new DriveToFieldLocationCommand(swerve,
            new Pose2d(2.4, 4, Rotation2d.fromDegrees(0))));
    addCommands(new DriveToTowerCommand(swerve, vision, true));

    // 15 secs to aligned

    }

}
