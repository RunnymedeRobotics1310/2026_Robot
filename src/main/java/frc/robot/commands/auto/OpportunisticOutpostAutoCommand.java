package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.ShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class OpportunisticOutpostAutoCommand extends SequentialCommandGroup {

  public OpportunisticOutpostAutoCommand(
      SwerveSubsystem swerve, HopperSubsystem hopper, LimelightVisionSubsystem vision) {
    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, 0, 0).withTimeout(0.8));

    addCommands(
        new ShooterCommand(hopper, swerve)
            .alongWith(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(10));

    addCommands(
        new DriveToFieldLocationCommand(swerve, new Pose2d(0.8, 0.6, Rotation2d.fromDegrees(0))));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, -0.5, 0, 0).withTimeout(1));

    addCommands(new NullDriveCommand(swerve).withTimeout(3));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 1, 1.6, 0).withTimeout(2));

    addCommands(
        new ShooterCommand(hopper, swerve)
            .alongWith(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(10));

    /*
    addCommands(
        new DriveToFieldLocationCommand(swerve, new Pose2d(2.4, 4, Rotation2d.fromDegrees(180))));
    addCommands(new DriveToTowerCommand(swerve, vision, false));

     */
  }
}
