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
        new LazyShooterCommand(hopper, 4000, 0, 5).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(
        new DriveRobotOrientedAtHeadingCommand(swerve, -1, 0, 0)
            .withTimeout(1.6)
            .andThen(new DriveRobotOrientedAtHeadingCommand(swerve, 0, 0, 180).withTimeout(1))
            .alongWith(new AutoClimbCommand(climb, hopper, true)));

    addCommands(
        new DriveToTowerCommand(swerve, vision, true)
            .andThen(new AutoClimbCommand(climb, hopper, false)));
  }
}
