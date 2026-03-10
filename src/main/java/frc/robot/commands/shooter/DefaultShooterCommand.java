package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.ACCPETED_SHOOTER_ERROR;
import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.CLOSE_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.SUPER_FAR_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.SUPER_FAR_SHOOT_HOOD_VALUE;

import ca.team1310.swerve.math.SwerveMath;
import ca.team1310.swerve.utils.SwerveUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DefaultShooterCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final OperatorInput oi;
  private final ShooterTuneNTBridge bridge;

  private boolean firstShot = false;
  private int logCounter = 0;
  private double lastHoodAngle = -1;
  private boolean wasKickerEnabled = false;

  public DefaultShooterCommand(
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      OperatorInput oi,
      ShooterTuneNTBridge bridge) {

    this.shooterSubsystem = shooter;
    this.swerveSubsystem = swerve;
    this.oi = oi;
    this.bridge = bridge;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    if (oi.shootFromAnywhere()) shooting();
    else if (bridge.isDashboardConnected()) {
      handleShooterBridge();
    } else {
      firstShot = false;
      shooterSubsystem.setHood(0);
      shooterSubsystem.stop();
    }

    // operator overrides
    if (oi.isRunAgitator()) shooterSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    if (oi.isReverseKicker()) shooterSubsystem.setKickerSpeed(-KICKER_RUNSPEED);
    if (oi.isStopFlywheel()) shooterSubsystem.setShooterSpeed(-0.01);
    if (oi.isReverseFlywheel()) shooterSubsystem.setShooterSpeed(-0.1);
    if (oi.putHoodDown()) shooterSubsystem.setHood(0);
  }

  public void shooting() {
    // shooter
    double distance = swerveSubsystem.distanceToHub();
    double targetSpeed = shooterSubsystem.calculateShootingSpeed(distance);
    shooterSubsystem.setShooterVelocity(targetSpeed);

    // agitator
    shooterSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);

    // hood
    if (distance < MAX_SHOOTING_DISTANCE) {
      if (distance >= SUPER_FAR_SHOOTING_DISTANCE) {
        shooterSubsystem.setHood(SUPER_FAR_SHOOT_HOOD_VALUE);
      } else if (distance >= MEDIUM_SHOOTING_DISTANCE) {
        shooterSubsystem.setHood(MEDIUM_SHOOT_HOOD_VALUE);
      }
    } else {
      shooterSubsystem.setHood(CLOSE_SHOOT_HOOD_VALUE);
    }

    // kicker
    double currentVelocity = shooterSubsystem.getShooterVelocity();
    boolean atSpeed = Math.abs(targetSpeed - currentVelocity) < ACCPETED_SHOOTER_ERROR;
    boolean facingHub =
        SwerveUtils.isCloseEnough(
            swerveSubsystem.angleToHub().getDegrees(), swerveSubsystem.getYaw(), 5);

    if ((atSpeed || firstShot) && facingHub) {
      firstShot = true;
      shooterSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    } else {
      shooterSubsystem.setKickerSpeed(0);
    }
  }

  private void handleShooterBridge() {
    // Dashboard shooter tuning
    double targetRPM = bridge.getTargetRPM();
    double currentRPM = shooterSubsystem.getShooterVelocity();
    boolean shooterEnabled = bridge.isShooterEnabled();
    double hoodAngle = bridge.getHoodAngle();

    if (shooterEnabled) {
      shooterSubsystem.setShooterVelocity(targetRPM);
    } else {
      shooterSubsystem.setShooterVelocity(0);
    }

    if (hoodAngle != lastHoodAngle) {
      shooterSubsystem.setHood(hoodAngle);
      lastHoodAngle = hoodAngle;
    }

    boolean atSpeed =
        shooterEnabled
            && targetRPM > 0
            && Math.abs(targetRPM - currentRPM) < ACCPETED_SHOOTER_ERROR;
    bridge.setCurrentRPM(currentRPM);
    bridge.setAtSpeed(atSpeed);
    bridge.setDistanceToHub(swerveSubsystem.distanceToHub());
    double headingError =
        SwerveMath.normalizeDegrees(
            swerveSubsystem.angleToHub().getDegrees() - swerveSubsystem.getYaw());
    bridge.setAngleToHub(headingError);

    boolean kickerEnabled = bridge.isKickerEnabled();
    if (kickerEnabled) {
      shooterSubsystem.setKickerSpeed(-bridge.getKickerSpeed());
      shooterSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    } else if (wasKickerEnabled) {
      shooterSubsystem.setKickerSpeed(0);
      shooterSubsystem.setAgitatorSpeed(0);
    }
    wasKickerEnabled = kickerEnabled;

    if (logCounter++ % 50 == 0) {
      log(
          "tune: target="
              + targetRPM
              + " current="
              + format(currentRPM)
              + " enabled="
              + shooterEnabled
              + " hood="
              + hoodAngle
              + " kicker="
              + bridge.isKickerEnabled());
    }
  }
}
