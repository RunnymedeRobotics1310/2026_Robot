package frc.robot.commands.auto.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ConfigFaceFieldPointCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final double targetXMetres;
    private final double targetYMetres;
    private final double headingToleranceDegrees;
    private Translation2d allianceTargetPoint;

    public ConfigFaceFieldPointCommand(
            SwerveSubsystem swerve,
            double targetXMetres,
            double targetYMetres,
            double headingToleranceDegrees) {
        this.swerve = swerve;
        this.targetXMetres = targetXMetres;
        this.targetYMetres = targetYMetres;
        this.headingToleranceDegrees = headingToleranceDegrees;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        allianceTargetPoint = new Translation2d(targetXMetres, targetYMetres);
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            allianceTargetPoint = RunnymedeUtils.getRedAllianceLocation(allianceTargetPoint);
        }
        logCommandStart("target=(" + targetXMetres + "," + targetYMetres + ") tol=" + headingToleranceDegrees);
    }

    @Override
    public void execute() {
        double targetHeading = targetHeadingDegrees();
        double omega = swerve.computeOmega(targetHeading);
        swerve.driveFieldOriented(0, 0, omega);
    }

    @Override
    public boolean isFinished() {
        double targetHeading = targetHeadingDegrees();
        double error = normalizeDelta(targetHeading - swerve.getYaw());
        if (Math.abs(error) <= headingToleranceDegrees) {
            setFinishReason("Heading reached");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        logCommandEnd(interrupted);
    }

    private double targetHeadingDegrees() {
        double dx = allianceTargetPoint.getX() - swerve.getPose().getX();
        double dy = allianceTargetPoint.getY() - swerve.getPose().getY();
        return new Rotation2d(dx, dy).getDegrees();
    }

    private double normalizeDelta(double deltaDegrees) {
        double error = deltaDegrees;
        while (error > 180) {
            error -= 360;
        }
        while (error < -180) {
            error += 360;
        }
        return error;
    }
}
