package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.LazyShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class SimpleCenterAutoCommand extends SequentialCommandGroup {
  //
  public SimpleCenterAutoCommand(
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      LimelightVisionSubsystem vision,
      ClimbSubsystem climb,
      double delay) {

    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(
        new LazyShooterCommand(hopper, 3400, 0, 5).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, -2, 0, 0).withTimeout(0.8));
    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 0, 0, 180).withTimeout(1));

    addCommands(new DriveToTowerCommand(swerve, vision, false));

    addCommands(new AutoClimbCommand(climb, hopper, true).withTimeout(1));

    addCommands(new AutoClimbCommand(climb, hopper, false).withTimeout(1));
  }
}
