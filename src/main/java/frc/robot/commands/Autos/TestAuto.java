package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeading;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto(SwerveSubsystem swerve, double delay) {
    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new DriveRobotOrientedAtHeading(swerve, 1, 1, 0).withTimeout(0.5));
  }
}
