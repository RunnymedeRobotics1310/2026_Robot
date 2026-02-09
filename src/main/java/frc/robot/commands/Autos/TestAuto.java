package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeading;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto(SwerveSubsystem swerve, double delay) {
    addCommands(new WaitCommand(delay));

    addCommands(new DriveRobotOrientedAtHeading(swerve, 1, 1, 0).withTimeout(0.8));
  }
}
