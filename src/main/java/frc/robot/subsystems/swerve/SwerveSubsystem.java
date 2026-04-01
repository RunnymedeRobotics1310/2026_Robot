package frc.robot.subsystems.swerve;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.utils.SwerveUtils;
import ca.team1310.swerve.vision.LimelightAwareSwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.telemetry.Telemetry;

public class SwerveSubsystem extends SubsystemBase {

  private final RunnymedeSwerveDrive drive;
  private final SwerveDriveSubsystemConfig config;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter omegaLimiter;
  private final PIDController headingPIDController;

  private double distanceToHub = 0;

  public SwerveSubsystem(SwerveDriveSubsystemConfig config) {
    this.drive =
        new LimelightAwareSwerveDrive(
            config.coreConfig(), config.gyroConfig(), config.limelightConfig());
    Telemetry.swerve = drive.getSwerveTelemetry();
    this.config = config;
    this.xLimiter = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
    this.yLimiter = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
    this.omegaLimiter = new SlewRateLimiter(config.rotationConfig().maxAccelerationRadPS2());
    headingPIDController =
        new PIDController(
            config.rotationConfig().headingP(),
            config.rotationConfig().headingI(),
            config.rotationConfig().headingD());
    headingPIDController.enableContinuousInput(-180, 180);
    headingPIDController.setTolerance(1);
    Telemetry.drive.enabled = config.telemetryEnabled();
  }

  @Override
  public void periodic() {
    distanceToHub = calculateDistanceToHub();
    Telemetry.drive.distanceToHub = distanceToHub;

    Telemetry.drive.robotRoll = drive.getRoll();
    Telemetry.drive.robotPitch = drive.getPitch();

    SmartDashboard.putNumber("1310/Roll", drive.getRoll());
    SmartDashboard.putNumber("1310/Pitch", drive.getPitch());

    if (Constants.TelemetryConfig.odometryDebugEnabled) {
      // Update odometry debug telemetry with current poses
      // Note: Field2d visualization is already provided by RunnymedeSwerve's FieldAwareSwerveDrive
      Telemetry.odometryDebug.updateOdometryPose(getPose());
      Telemetry.odometryDebug.updateWheelOnlyPose(drive.getWheelOnlyPose());
    }

    Telemetry.drive.robotPose =
        new Pose2d(drive.getPose().getTranslation(), Rotation2d.fromDegrees(drive.getYaw()));
  }

  /*
   * *********************************************************************************************
   * Core methods for controlling the drivebase
   */

  /**
   * Add limiters to the change in drive values. Note this may not scale evenly - one may reach
   * desired speed before another.
   *
   * @param x m/s
   * @param y m/s
   * @param omega rad/s
   */
  private void driveSafely(double x, double y, double omega) {
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    omega =
        SwerveUtils.clamp(
            -config.rotationConfig().maxRotVelocityRadPS(),
            omega,
            config.rotationConfig().maxRotVelocityRadPS());
    omega = omegaLimiter.calculate(omega);

    if (this.config.enabled()) {
      this.drive.driveRobotOriented(x, y, omega);
    }
  }

  /**
   * Add limiters to the change in drive values. Note this may not scale evenly - one may reach
   * desired speed before another.
   *
   * @param x m/s
   * @param y m/s
   * @param omega rad/s
   */
  private void driveSafelyFieldOriented(double x, double y, double omega) {
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    omega =
        SwerveUtils.clamp(
            -config.rotationConfig().maxRotVelocityRadPS(),
            omega,
            config.rotationConfig().maxRotVelocityRadPS());
    omega = omegaLimiter.calculate(omega);

    if (this.config.enabled()) {
      this.drive.driveFieldOriented(x, y, omega);
    }
  }

  /**
   * The primary method for controlling the drivebase. The provided parameters specify the
   * robot-relative chassis speeds of the robot.
   *
   * <p>This method is responsible for applying safety code to prevent the robot from attempting to
   * exceed its physical limits both in terms of speed and acceleration.
   *
   * <p>
   *
   * @param x m/s
   * @param y m/s
   * @param omega rad/s
   */
  public final void driveRobotOriented(double x, double y, double omega) {
    Telemetry.drive.fieldOrientedVelocityX = 0;
    Telemetry.drive.fieldOrientedVelocityY = 0;
    Telemetry.drive.fieldOrientedVelocityOmega = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseX = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseY = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseHeading = 0;

    driveSafely(x, y, omega);
  }

  /** Stop all motors as fast as possible */
  public void stop() {
    driveRobotOriented(0, 0, 0);
  }

  /**
   * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
   * field-oriented inputs into the required robot-oriented inputs that can be used by the robot.
   *
   * @param x the linear velocity of the robot in metres per second. Positive x is away from the
   *     blue alliance wall
   * @param y the linear velocity of the robot in metres per second. Positive y is to the left of
   *     the robot
   * @param omega the rotation rate of the heading of the robot in radians per second. CCW positive.
   */
  public final void driveFieldOriented(double x, double y, double omega) {
    Telemetry.drive.fieldOrientedDeltaToPoseX = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseY = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseHeading = 0;
    Telemetry.drive.fieldOrientedVelocityX = x;
    Telemetry.drive.fieldOrientedVelocityY = y;
    Telemetry.drive.fieldOrientedVelocityOmega = omega;

    driveSafelyFieldOriented(x, y, omega);
  }

  /**
   * Lock the swerve drive to prevent it from moving. This can only be called when the robot is
   * nearly stationary.
   *
   * @return true if successfully locked, false otherwise
   */
  public boolean lock() {
    return drive.lock();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return drive.getPose(); // todo: fixme:
  }

  /**
   * Update the vision pose for odometry debugging telemetry. This should be called by the vision
   * subsystem when a new vision measurement is available.
   *
   * <p>Note: Vision pose fusion into odometry is handled by RunnymedeSwerve's
   * LimelightAwareSwerveDrive.
   *
   * @param visionPose the vision-measured pose
   * @param timestamp the timestamp of the vision measurement
   * @param isValid whether the vision pose is valid
   */
  public void updateVisionPose(Pose2d visionPose, double timestamp, boolean isValid) {
    if (Constants.TelemetryConfig.odometryDebugEnabled) {
      Telemetry.odometryDebug.updateVisionPose(visionPose, timestamp, isValid);
    }
  }

  public double getYaw() {
    return drive.getYaw();
  }

  public double getYawRate() {
    return drive.getYawRate();
  }

  /**
   * Get the wrapped heading error (desired-current) in degrees, normalized to [-180, 180].
   *
   * @param desiredHeadingDegrees desired heading in degrees
   * @return heading error in degrees
   */
  public double getHeadingErrorDegrees(double desiredHeadingDegrees) {
    return MathUtil.inputModulus(desiredHeadingDegrees - drive.getYaw(), -180, 180);
  }

  /**
   * Get the measured translational speed of the robot in metres per second from swerve kinematics.
   *
   * @return translational speed in m/s
   */
  public double getMeasuredTranslationSpeedMPS() {
    double[] velocity = drive.getMeasuredRobotVelocity();
    if (velocity == null || velocity.length < 2) {
      return 0;
    }
    return Math.hypot(velocity[0], velocity[1]);
  }

  /**
   * Set the gyro yaw offset of the robot, in degrees.
   *
   * @param yaw the yaw offset of the robot, in degrees
   */
  public void setYaw(double yaw) {
    drive.setYaw(yaw);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    drive.zeroGyro();
  }

  /**
   * Change the robot's internal understanding of its position and rotation. This is not an
   * incremental change or suggestion, it discontinuously re-sets the pose to the specified pose.
   *
   * @param pose the new location and heading of the robot.
   */
  public void resetOdometry(Pose2d pose) {
    drive.resetOdometry(pose);
  }

  /**
   * Set the desired module state for the named module. This should ONLY be used when testing the
   * serve drivebase in a controlled environment.
   *
   * <p>This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY!
   *
   * @param moduleName the module to activate
   * @param speed in m/s
   * @param angle in degrees
   */
  public void setModuleState(String moduleName, double speed, double angle) {
    drive.setModuleState(moduleName, speed, angle);
  }

  @Override
  public String toString() {
    Pose2d pose = getPose();
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getDegrees();

    StringBuilder sb = new StringBuilder();
    sb.append(this.getClass().getSimpleName())
        .append(": Pose: ")
        .append(Math.round(x * 100d) / 100d)
        .append(",")
        .append(Math.round(y * 100d) / 100d)
        .append(",")
        .append(Math.round(theta * 10d) / 10d);

    return sb.toString();
  }

  /*
   * *********************************************************************************************
   * Convenience methods for subsystem users
   */

  /**
   * Compute the required rotation speed of the robot given the desired heading. Note the desired
   * heading is specified in degrees, adn the returned value is in radians per second.
   *
   * @param desiredHeadingDegrees the desired heading of the robot
   * @return the required rotation speed of the robot (omega) in rad/s
   */
  public double computeOmega(double desiredHeadingDegrees) {
    return computeOmega(desiredHeadingDegrees, config.rotationConfig().defaultRotVelocityRadPS());
  }

  /**
   * Compute the required rotation speed of the robot given the desired heading. Note the desired
   * heading is specified in degrees, adn the returned value is in radians per second.
   *
   * @param desiredHeadingDegrees the desired heading of the robot
   * @param maxOmegaRadPerSec the maximum allowable rotation speed of the robot
   * @return the required rotation speed of the robot (omega) in rad/s
   */
  public double computeOmega(double desiredHeadingDegrees, double maxOmegaRadPerSec) {
    double omega = headingPIDController.calculate(drive.getYaw(), desiredHeadingDegrees);
    return SwerveUtils.clamp(-maxOmegaRadPerSec, omega, maxOmegaRadPerSec);
  }

  public double oldComputeTranslateVelocity(double distance, double maxSpeedMPS, double tolerance) {
    final double decelZoneMetres = 1.2;
    final double verySlowZone = 0.2;
    final double verySlowSpeed = 0.15;
    double speed;

    final double absDist = Math.abs(distance);

    if (absDist < tolerance) {
      return 0;
    }
    // Very slow zone
    if (absDist < verySlowZone) {
      speed = verySlowSpeed;
    }
    // Full speed ahead!
    else if (absDist >= decelZoneMetres) {
      speed = maxSpeedMPS;
    }
    // Decel Zone
    else {
      speed = (absDist / decelZoneMetres) * maxSpeedMPS;
    }

    speed *= Math.signum(distance);

    return speed;
  }

  public Translation2d computeVelocity(Translation2d translationToTravel, double maxSpeed) {

    double distanceMetres = translationToTravel.getNorm();

    // don't worry about tiny translations
    if (distanceMetres < config.translationConfig().toleranceMetres()) {
      return new Translation2d();
    }

    // safety code
    if (maxSpeed > config.translationConfig().maxSpeedMPS()) {
      maxSpeed = config.translationConfig().maxSpeedMPS();
    }

    double speed = Math.min(maxSpeed, distanceMetres * maxSpeed / 2);

    // Confirm speed is not too slow to move
    if (speed < config.translationConfig().minSpeedMPS()) {
      speed = config.translationConfig().minSpeedMPS();
    }

    Rotation2d angle = translationToTravel.getAngle();

    double xSign = Math.signum(translationToTravel.getX());
    double ySign = Math.signum(translationToTravel.getY());
    return new Translation2d(
        xSign * speed * Math.abs(angle.getCos()), ySign * speed * Math.abs(angle.getSin()));
  }

  /**
   * Finds the angle to aim to to shoot - either the Hub if we're in the zone, or the side of our
   * zone away from the hub if we're in the ball pit
   *
   * @return Angle to shooting destination
   */
  public Rotation2d angleToShootTowards() {
    Pose2d pose = getPose();
    Translation2d hubPose = Constants.FieldLocation.HUB_CENTRE.getLocation();

    // if past alliance zone, point at trench
    boolean pastHub = RunnymedeUtils.isFurtherThan(pose, hubPose);

    if (pastHub) {
      if (RunnymedeUtils.isLeftOf(pose, hubPose)) {
        hubPose = Constants.FieldLocation.ZONE_SHOTS_LEFT.getLocation();
      } else {
        hubPose = Constants.FieldLocation.ZONE_SHOTS_RIGHT.getLocation();
      }
    }

    double dx, dy;
    dx = hubPose.getX() - pose.getX();
    dy = hubPose.getY() - pose.getY();

    return new Rotation2d(dx, dy);
  }

  public double calculateDistanceToHub() {
    Pose2d pose = getPose();
    Translation2d hubPose =
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      hubPose = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));
    }

    return hubPose.getDistance(pose.getTranslation());
  }

  public double distanceToHub() {
    return distanceToHub;
  }
}
