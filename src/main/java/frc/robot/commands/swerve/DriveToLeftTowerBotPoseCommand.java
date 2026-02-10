package frc.robot.commands.swerve;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToLeftTowerBotPoseCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final LimelightVisionSubsystem vision;

    public DriveToLeftTowerBotPoseCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }
}
