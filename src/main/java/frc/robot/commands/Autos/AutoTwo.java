package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoTwo extends SequentialCommandGroup {

  public AutoTwo(SwerveSubsystem swerve, double delay) {
    addCommands(new WaitCommand(delay));

    addCommands(
        new TestAuto(swerve, delay)
            .withTimeout(0.5)
            .andThen(new WaitCommand(1))
            .andThen(new TestAuto(swerve, delay)));
  }
}
