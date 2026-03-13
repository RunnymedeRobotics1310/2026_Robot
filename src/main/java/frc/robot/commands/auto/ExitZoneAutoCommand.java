package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeadingCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

  public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {
    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new SetAllianceGyroCommand(swerve, 180));

    addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 0.50, 0.00, 180).withTimeout(0.55));
  }
}
