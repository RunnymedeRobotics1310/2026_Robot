package frc.robot.commands.swerve;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(
    value = "drive_to_pose",
    category = "drive",
    description = "Drive to a field pose using odometry")
public class DriveToFieldLocationAimedAtHubCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final Pose2d location;
  private final double tolerance;
  private final double headingToleranceDegrees;
  private final double timeoutSeconds;
  private Pose2d allianceLocation;
  private double targetHeadingDeg;
  private double maxSpeedMPS;

  public DriveToFieldLocationAimedAtHubCommand(SwerveSubsystem swerve, Pose2d pose) {
    this(swerve, pose, 0.05, 2.0, 0);
    this.maxSpeedMPS = Constants.Swerve.TRANSLATION_CONFIG.maxSpeedMPS();
  }

  public DriveToFieldLocationAimedAtHubCommand(
      SwerveSubsystem swerve, Pose2d pose, double toleranceM) {
    this(swerve, pose, toleranceM, 2.0, 0);
    this.maxSpeedMPS = Constants.Swerve.TRANSLATION_CONFIG.maxSpeedMPS();
  }

  public DriveToFieldLocationAimedAtHubCommand(
      SwerveSubsystem swerve, Pose2d pose, double toleranceM, double maxSpeedMPS) {
    this(swerve, pose, toleranceM, toleranceM, 0);
    this.maxSpeedMPS = maxSpeedMPS;
  }

  public DriveToFieldLocationAimedAtHubCommand(
      SwerveSubsystem swerve,
      @ConfigParam(
              value = "xMetres",
              unit = "m",
              min = 0,
              max = 16.54,
              description = "Target X position on field")
          double xMetres,
      @ConfigParam(
              value = "yMetres",
              unit = "m",
              min = 0,
              max = 8.07,
              description = "Target Y position on field")
          double yMetres,
      @ConfigParam(
              value = "headingDegrees",
              unit = "deg",
              description = "Target heading at destination")
          double headingDegrees,
      @ConfigParam(
              value = "positionToleranceMetres",
              unit = "m",
              min = 0,
              max = 1,
              defaultValue = 0.05,
              description = "Position tolerance")
          double positionToleranceMetres,
      @ConfigParam(
              value = "headingToleranceDegrees",
              unit = "deg",
              min = 0,
              max = 30,
              defaultValue = 3.0,
              description = "Heading tolerance")
          double headingToleranceDegrees,
      @ConfigParam(
              value = "timeoutSeconds",
              unit = "s",
              min = 0,
              max = 15,
              description = "Safety timeout")
          double timeoutSeconds) {
    this(
        swerve,
        new Pose2d(xMetres, yMetres, Rotation2d.fromDegrees(headingDegrees)),
        positionToleranceMetres,
        headingToleranceDegrees,
        timeoutSeconds);
  }

  private DriveToFieldLocationAimedAtHubCommand(
      SwerveSubsystem swerve,
      Pose2d pose,
      double tolerance,
      double headingToleranceDegrees,
      double timeoutSeconds) {
    this.swerve = swerve;
    this.location = pose;
    this.tolerance = tolerance;
    this.headingToleranceDegrees = headingToleranceDegrees;
    this.timeoutSeconds = timeoutSeconds;
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
    Translation2d transV = swerve.computeVelocity(dif, maxSpeedMPS);

    //    double angleDif =
    //        SwerveUtils.normalizeDegrees(targetHeadingDeg -
    // currentPose.getRotation().getDegrees());

    //    double maxOmega = Math.max((Math.toRadians(angleDif) / dif.getNorm()) * transV.getNorm(),
    // .1);
    double omega = swerve.computeOmega(swerve.getHubAngleDeg());
    //    System.out.println(maxOmega);

    // face hub ff (to face hub while moving quickly)
    double vTan =
        transV.getNorm()
            * Math.sin(Math.toRadians(swerve.getHubAngleDeg()) - transV.getAngle().getRadians());
    double omegaFFToHub = vTan / swerve.distanceToHub();
    omega += omegaFFToHub;

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
            swerve.getPose().getTranslation(), allianceLocation.getTranslation(), tolerance));
    //            && SwerveUtils.isCloseEnough(swerve.getYaw(), targetHeadingDeg, 2));
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
