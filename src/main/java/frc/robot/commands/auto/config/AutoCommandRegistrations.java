package frc.robot.commands.auto.config;

import frc.robot.commands.auto.AutoClimbCommand;
import frc.robot.commands.hopper.LazyShooterCommand;
import frc.robot.commands.hopper.ShooterCommand;
import frc.robot.commands.swerve.DriveDistanceCommand;
import frc.robot.commands.swerve.DriveFieldOrientedCommand;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeadingCommand;
import frc.robot.commands.swerve.DriveTimedCommand;
import frc.robot.commands.swerve.DriveToFieldLocationCommand;
import frc.robot.commands.swerve.DriveToTowerCommand;
import frc.robot.commands.swerve.FaceFieldPointCommand;
import frc.robot.commands.swerve.FaceHubCommand;
import frc.robot.commands.swerve.HoldPositionCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.RotateToHeadingCommand;
import frc.robot.commands.swerve.SetPoseCommand;

public class AutoCommandRegistrations {

  public static void registerAll(AutoCommandRegistry registry) {
    registry.register(DriveDistanceCommand.class);
    registry.register(DriveTimedCommand.class);
    registry.register(DriveRobotOrientedAtHeadingCommand.class);
    registry.register(DriveToFieldLocationCommand.class);
    registry.register(RotateToHeadingCommand.class);
    registry.register(HoldPositionCommand.class);
    registry.register(SetPoseCommand.class);
    registry.register(FaceFieldPointCommand.class);
    registry.register(FaceHubCommand.class);
    registry.register(DriveToTowerCommand.class);
    registry.register(LazyShooterCommand.class);
    registry.register(ShooterCommand.class);
    registry.register(DriveFieldOrientedCommand.class);
    registry.register(NullDriveCommand.class);
    registry.register(AutoClimbCommand.class);
  }
}
