package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.ShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class RightShootClimbAutoCommand extends SequentialCommandGroup {

  public RightShootClimbAutoCommand(
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      LimelightVisionSubsystem vision,
      ClimbSubsystem climb,
      double delay) {

    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, 0, 0).withTimeout(0.8));

    addCommands(
        new ShooterCommand(hopper, swerve)
            .alongWith(new FaceHubCommand(swerve).andThen(new NullDriveCommand(swerve)))
            .withTimeout(7));

    addCommands(
        new DriveFieldOrientedCommand(swerve, 0, 1, 180)
            .withTimeout(1.5)
            .andThen(new DriveFieldOrientedCommand(swerve, -1, 0, 180).withTimeout(0.4))
            .alongWith(new AutoClimbCommand(climb, hopper, true)));

    addCommands(
        new DriveToTowerCommand(swerve, vision, true)
            .andThen(new AutoClimbCommand(climb, hopper, false)));
  }
}
