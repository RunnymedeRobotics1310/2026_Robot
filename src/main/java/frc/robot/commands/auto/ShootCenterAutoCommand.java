package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LazyIntakeCommand;
import frc.robot.commands.swerve.DriveFieldOrientedCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootCenterAutoCommand extends SequentialCommandGroup {

  public ShootCenterAutoCommand(SwerveSubsystem swerve, IntakeSubsystem intake, double delay) {

    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new SetAllianceGyroCommand(swerve, 180));

    addCommands(
        new DriveFieldOrientedCommand(swerve, 0, 1, 180)
            .withTimeout(0.3)); // Moves away from side of trench

    addCommands(
        new DriveFieldOrientedCommand(swerve, 1, 0, 180).withTimeout(2)); // Moves toward ball pit

    addCommands(
        new DriveFieldOrientedCommand(swerve, 0, -1, 180)
            .withTimeout(3)
            .alongWith(
                new LazyIntakeCommand(intake).withTimeout(3))); // Goes into ball pit and intakes
  }
}
