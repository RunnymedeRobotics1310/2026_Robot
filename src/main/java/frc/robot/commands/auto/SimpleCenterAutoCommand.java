package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.LazyShooterCommand;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeadingCommand;
import frc.robot.commands.swerve.DriveToTowerCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class SimpleCenterAutoCommand extends SequentialCommandGroup {
  //
  public SimpleCenterAutoCommand(
      SwerveSubsystem swerve, ShooterSubsystem shooter, LimelightVisionSubsystem vision) {

    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(
        new LazyShooterCommand(shooter, 2800, 0, 10).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, -2, 0.7, 0).withTimeout(1));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 0, 0, 180).withTimeout(2));

    addCommands(new DriveToTowerCommand(swerve, vision, true));
  }
}
