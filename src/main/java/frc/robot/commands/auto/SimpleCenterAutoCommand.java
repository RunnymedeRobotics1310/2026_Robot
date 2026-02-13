package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.DriveRobotOrientedAtHeadingCommand;
import frc.robot.commands.swerve.DriveToLeftTowerCommand;
import frc.robot.commands.swerve.NullDriveCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class SimpleCenterAutoCommand extends SequentialCommandGroup {

    public SimpleCenterAutoCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision) {

        addCommands(new SetAllianceGyroCommand(swerve, 180));
        addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 1, 0, 180)
                .withTimeout(1));

        addCommands(new NullDriveCommand(swerve).withTimeout(3));

        addCommands(new DriveRobotOrientedAtHeadingCommand(swerve, 0, 0, 0)
                .withTimeout(2));

        addCommands(new DriveToLeftTowerCommand(swerve, vision));

        // 9 secs to aligned
    }

}
