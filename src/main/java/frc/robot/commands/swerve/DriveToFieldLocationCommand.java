package frc.robot.commands.swerve;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToFieldLocationCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final Pose2d location;
  private Pose2d allianceLocation;
  private double targetHeadingDeg;
  private double tolerance = 0.05;

  public DriveToFieldLocationCommand(SwerveSubsystem swerve, Pose2d pose) {
    this.swerve = swerve;
    this.location = pose;
    addRequirements(swerve);
  }

  public DriveToFieldLocationCommand(
      SwerveSubsystem swerve, Pose2d pose, double toleranceM) {
    this.swerve = swerve;
    this.location = pose;
    this.tolerance = toleranceM;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      this.allianceLocation = RunnymedeUtils.getRedAlliancePose(location);
    } else {
      this.allianceLocation = location;
    }
    this.targetHeadingDeg =
        SwerveUtils.normalizeDegrees(allianceLocation.getRotation().getDegrees());

    log("Pose: " + swerve.getPose() + " AllianceLoc:" + allianceLocation);
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double xDif = allianceLocation.getX() - currentPose.getX();
    double yDif = allianceLocation.getY() - currentPose.getY();
    Translation2d dif = new Translation2d(xDif, yDif);
    Translation2d transV =
        swerve.computeVelocity(dif, Constants.Swerve.TRANSLATION_CONFIG.maxSpeedMPS());

    double angleDif =
        SwerveUtils.normalizeDegrees(targetHeadingDeg - currentPose.getRotation().getDegrees());

    double maxOmega = Math.max((Math.toRadians(angleDif) / dif.getNorm()) * transV.getNorm(), .1);
    double omega = swerve.computeOmega(targetHeadingDeg, maxOmega);
    System.out.println(maxOmega);

    swerve.driveFieldOriented(transV.getX(), transV.getY(), omega);
  }

  @Override
  public boolean isFinished() {
    //        return (SwerveUtils.isCloseEnough(
    //                swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.05)
    //                && SwerveUtils.isCloseEnough(swerve.getPose().getRotation().getDegrees(),
    // targetHeadingDeg, 10));
    boolean done =
        (SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(), allianceLocation.getTranslation(), tolerance)
            && SwerveUtils.isCloseEnough(swerve.getYaw(), targetHeadingDeg, 2));
    if (done) {
      System.out.println(
          "REACHED DESTINATION: x["
              + swerve.getPose().getX()
              + "] y["
              + swerve.getPose().getY()
              + "], deg["
              + swerve.getPose().getRotation().getDegrees()
              + "]");
    }
    return done;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
  }
}
