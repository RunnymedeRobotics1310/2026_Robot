package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.AutoShooterCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class RightShootClimbAutoCommand extends SequentialCommandGroup {

  public RightShootClimbAutoCommand(
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      LimelightVisionSubsystem vision,
      ClimbSubsystem climb,
      double delay) {

    addCommands(new SetAllianceGyroCommand(swerve, 0));

    addCommands(new NullDriveCommand(swerve).withTimeout(delay));

    addCommands(new DriveFieldOrientedCommand(swerve, -2, 0, 0).withTimeout(1));

    addCommands(
        ((new AutoShooterCommand(hopper, swerve, 16).withTimeout(5))
            .deadlineFor(
                new DriveToFieldLocationAimedAtHubCommand(
                        swerve, new Pose2d(2.2, 4.0, new Rotation2d(0)), 0.3, 0.3)
                    .andThen(new FaceHubCommand(swerve, true)))));

    addCommands(new FaceAngleCommand(swerve, Rotation2d.fromDegrees(180)));

    addCommands(
        (new AutoClimbCommand(climb, hopper, true)
                .alongWith(new DriveToTowerCommand(swerve, vision, climb, true)))
            .andThen(new AutoClimbCommand(climb, hopper, false)));
  }
}
