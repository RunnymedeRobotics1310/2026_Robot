package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FaceHubCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;

    public FaceHubCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        Rotation2d hubAngle = swerve.angleToHub() ;
        double omega = swerve.computeOmega(hubAngle.getDegrees() + 180);
        swerve.driveFieldOriented(0, 0, omega);
    }

    @Override
    public boolean isFinished() {
        double error = Math.abs(swerve.angleToHub().getDegrees() - swerve.getYaw());
        log("Error: " + error);
        return error <= 3;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        swerve.stop();
    }
}
