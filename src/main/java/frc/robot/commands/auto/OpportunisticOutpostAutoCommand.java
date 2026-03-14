package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.ShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class OpportunisticOutpostAutoCommand extends SequentialCommandGroup {

  public OpportunisticOutpostAutoCommand(
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      LimelightVisionSubsystem vision,
      double delay) {
    addCommands(new SetAllianceGyroCommand(swerve, 0));
    addCommands(new SetPoseCommand(swerve, 3.4, 2.2, 0));

    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, 0, 0).withTimeout(0.8));

    addCommands(
        new ShooterCommand(hopper, swerve)
            .alongWith(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(6));

    addCommands(new DriveFieldOrientedCommand(swerve, 0, 0, 180).withTimeout(1));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, -1.7, 0).withTimeout(1.6));

    addCommands(new NullDriveCommand(swerve).withTimeout(5));

    addCommands(new DriveFieldOrientedCommand(swerve, 2, 2, 180).withTimeout(0.9));

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
