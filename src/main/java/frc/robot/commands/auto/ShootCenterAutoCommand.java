package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeadingCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootCenterAutoCommand extends SequentialCommandGroup {

  public ShootCenterAutoCommand(SwerveSubsystem swerve, double delay) {

      addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new SetAllianceGyroCommand(swerve, 180));
    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, -0.5, 0, 0).withTimeout(0.1));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 0, 0.5, 0).withTimeout(1.5));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 0.5, 0, 90).withTimeout(2));

  }
}
