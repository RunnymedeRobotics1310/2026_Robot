package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LazyIntakeCommand;
import frc.robot.commands.hopper.AutoShooterCommand;
import frc.robot.commands.swerve.DriveFieldOrientedCommand;
import frc.robot.commands.swerve.FaceHubCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
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
            .alongWith(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(7));

    addCommands(new DriveFieldOrientedCommand(swerve, -0.8, -0.1, 90).withTimeout(2));

    addCommands(
        new DriveFieldOrientedCommand(swerve, -0.5, 0, 90)
            .withTimeout(1)
            .deadlineFor(new LazyIntakeCommand(hopper)));

    addCommands(new DriveFieldOrientedCommand(swerve, 2, 0, 0).withTimeout(1.25));

    addCommands(
        new AutoShooterCommand(hopper, swerve, 8)
            .alongWith(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(7));
  }
}
