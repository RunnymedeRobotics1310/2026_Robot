package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.LazyShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class SimpleCenterAutoCommand extends SequentialCommandGroup {
  //
  public SimpleCenterAutoCommand(
      SwerveSubsystem swerve, HopperSubsystem hopper, LimelightVisionSubsystem vision) {

    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(
        new LazyShooterCommand(hopper, 3400, 0, 7).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, 0, 0).withTimeout(0.3));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, -1, 0.5, 180).withTimeout(2));

    addCommands(new DriveToTowerCommand(swerve, vision, false));
  }
}
