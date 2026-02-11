package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoTwo extends SequentialCommandGroup {

  public AutoTwo(SwerveSubsystem swerve, double delay) {
    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new TestAuto(swerve, delay).withTimeout(0.5).andThen(new TestAuto(swerve, delay)));
  }
}
