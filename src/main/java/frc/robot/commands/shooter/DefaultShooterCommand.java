package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.CLOSE_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.SUPER_FAR_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.SUPER_FAR_SHOOT_HOOD_VALUE;

import ca.team1310.swerve.utils.SwerveUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DefaultShooterCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final OperatorInput oi;

  private boolean firstShot = false;

  public DefaultShooterCommand(ShooterSubsystem shooter, SwerveSubsystem swerve, OperatorInput oi) {
    this.shooterSubsystem = shooter;
    this.swerveSubsystem = swerve;
    this.oi = oi;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    if (oi.shootFromAnywhere()) shooting();
    else {
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
}
