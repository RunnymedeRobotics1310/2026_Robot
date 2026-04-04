package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LazyIntakeCommand;
import frc.robot.commands.hopper.AutoShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DepotAutoCommand extends SequentialCommandGroup {
  public DepotAutoCommand(
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      LimelightVisionSubsystem vision,
      ClimbSubsystem climb,
      double delay) {

    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, 0, 0).withTimeout(0.8));

    addCommands(
        new AutoShooterCommand(hopper, swerve, 8)
            .deadlineFor(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(7));

    addCommands(
        new DriveToFieldLocationCommand(swerve, new Pose2d(1.1, 5.8, new Rotation2d(0)))
            .withTimeout(4));

    addCommands(new FaceAngleCommand(swerve, Rotation2d.fromDegrees(90)));
    addCommands(
        new DriveFieldOrientedCommand(swerve, -0.6, 0, 90)
            .withTimeout(1.5)
            .deadlineFor(new LazyIntakeCommand(hopper)));
    addCommands(new DriveFieldOrientedCommand(swerve, 1, 0, 90).withTimeout(2));
    addCommands(new FaceHubCommand(swerve));

    // holy moly, this hurts to look at (i made it)
    addCommands(
        ((new DriveToFieldLocationAimedAtHubCommand(
                    swerve, new Pose2d(2.2, 4.1, new Rotation2d(0)), 0.2, 0.1)
                .withTimeout(4)
                .andThen(new NullDriveCommand(swerve).withTimeout(3)))
            .alongWith(new AutoShooterCommand(hopper, swerve, 16))
            .withTimeout(7)));

    addCommands(
        (new FaceAngleCommand(swerve, Rotation2d.fromDegrees(180))
            //                .alongWith(new AutoClimbCommand(climb, hopper, true))
            )
            .andThen(
                new DriveToTowerCommand(swerve, vision, false)
                //                    .andThen(new AutoClimbCommand(climb, hopper, false))
                ));

    //    addCommands(
    //        new DriveToTowerCommand(swerve, vision, false)
    //            .andThen(new AutoClimbCommand(climb, hopper, false)));
  }
}
