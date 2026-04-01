package frc.robot.commands.hopper;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;

import ca.team1310.swerve.math.SwerveMath;
import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DefaultHopperCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final ClimbSubsystem climb;
  private final OperatorInput oi;
  private final ShooterTuneNTBridge bridge;

  private boolean firstShot = false;
  private boolean firstIntake = false;
  private final Timer intakeTimer = new Timer();

  // Jeff's scary AI thingy
  private int logCounter = 0;
  private double lastHoodAngle = -1;
  private boolean wasKickerEnabled = false;

  // Test fixes
  private final Timer kickerDebounceTimer = new Timer();

  public DefaultHopperCommand(
      HopperSubsystem hopper,
      SwerveSubsystem swerve,
      ClimbSubsystem climb,
      OperatorInput oi,
      ShooterTuneNTBridge bridge) {

    this.hopperSubsystem = hopper;
    this.swerveSubsystem = swerve;
    this.climb = climb;
    this.oi = oi;
    this.bridge = bridge;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
    firstShot = false;
    firstIntake = false;
    intakeTimer.reset();
    intakeTimer.start();
    kickerDebounceTimer.reset();
    kickerDebounceTimer.start();
  }

  @Override
  public void execute() {

    /* ----- INTAKE ----- */
    if (oi.isIntakeDoingStuff() && climb.isClimbDown()) {
      //      hopperSubsystem.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
      //      hopperSubsystem.setDoorSetpoint(INTAKE_DOOR_ANGLE);
      hopperSubsystem.setDoorSpeed(DOOR_SPEED);
      firstIntake = true;

      if (intakeTimer.get() < 0.25 && firstIntake) {
        hopperSubsystem.setRollerSpeeds(0, 0);
      } else {
        hopperSubsystem.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
      }
    } else if (oi.shootFromAnywhere() || oi.isCloseShoot()) {
      hopperSubsystem.setRollerSpeeds(0, INTAKE_SPEED);
      hopperSubsystem.setDoorSetpoint(0);
    } else {
      hopperSubsystem.setDoorSetpoint(0);
      hopperSubsystem.setRollerSpeeds(0, 0);
      intakeTimer.reset();
    }

    /* ----- SHOOTER ----- */
    if (oi.shootFromAnywhere()) shooting();
    else if (bridge.isDashboardConnected()) {
      handleShooterBridge();
    } else {
      firstShot = false;
      hopperSubsystem.setHood(0);
      hopperSubsystem.setShooterVelocity(0);
      hopperSubsystem.setKickerSpeed(0);
      hopperSubsystem.setAgitatorSpeed(0);
    }
    //    if (oi.isIntakeDoingStuff()) hopperSubsystem.setAgitatorSpeed(0);
    //    if (oi.shootFromAnywhere()) hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);

    /* ----- OPERATOR OVERRIDES ----- */
    if (oi.isRunAgitator()) hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    if (oi.isStopAgitator()) hopperSubsystem.setAgitatorSpeed(0);
    if (oi.isReverseKicker()) hopperSubsystem.setKickerSpeed(-KICKER_RUNSPEED);
    if (oi.isRunKicker()) hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    if (oi.isStopFlywheel()) hopperSubsystem.setShooterSpeed(-0.01);
    if (oi.isReverseFlywheel()) hopperSubsystem.setShooterSpeed(-0.1);
    if (oi.putHoodDown()) hopperSubsystem.setHood(0);

    if (oi.isIntakeForwards()) hopperSubsystem.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
    if (oi.isIntakeReverse()) {
      hopperSubsystem.setRollerSpeeds(-INTAKE_SPEED, -INTAKE_SPEED);
    }
    if (oi.isOpenDoor()) hopperSubsystem.setDoorSpeed(DOOR_SPEED);
    if (oi.isCloseDoor() && !hopperSubsystem.getDoorClosed()) {
      hopperSubsystem.setDoorSpeed(-0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    firstIntake = false;
    firstShot = false;
    intakeTimer.reset();
    intakeTimer.stop();
    kickerDebounceTimer.reset();
    kickerDebounceTimer.stop();
    hopperSubsystem.setHood(0);
    hopperSubsystem.stop();
  }

  public void shooting() {
    // distance
    double distance = swerveSubsystem.distanceToHub();

    // hood
    hopperSubsystem.setHood(hopperSubsystem.calculateHoodValule(distance));

    // shooter
    double targetSpeed = hopperSubsystem.calculateShootingSpeed(distance);
    hopperSubsystem.setShooterVelocity(targetSpeed);

    // agitator
    //    hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    //    hopperSubsystem.reverseAgitator(1);
    hopperSubsystem.pulseAgitator(1);

    // kicker
    boolean facingHub =
        SwerveUtils.isCloseEnough(
            swerveSubsystem.angleToShootTowards().getDegrees(), swerveSubsystem.getYaw(), 5);

    if (hopperSubsystem.isShooterAtSpeed() && facingHub) {
      kickerDebounceTimer.reset();
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    } else if (kickerDebounceTimer.hasElapsed(0.1)) {
      hopperSubsystem.setKickerSpeed(0);
    }
  }

  private void handleShooterBridge() {
    // Dashboard shooter tuning
    double targetRPM = bridge.getTargetRPM();
    double currentRPM = hopperSubsystem.getRightShooterVelocity();
    boolean shooterEnabled = bridge.isShooterEnabled();
    double hoodAngle = bridge.getHoodAngle();

    if (shooterEnabled) {
      hopperSubsystem.setShooterVelocity(targetRPM);
    } else {
      hopperSubsystem.setShooterVelocity(0);
    }

    if (hoodAngle != lastHoodAngle) {
      hopperSubsystem.setHood(hoodAngle);
      lastHoodAngle = hoodAngle;
    }

    boolean atSpeed = shooterEnabled && targetRPM > 0 && hopperSubsystem.isShooterAtSpeed();
    bridge.setCurrentRPM(currentRPM);
    bridge.setAtSpeed(atSpeed);
    bridge.setDistanceToHub(swerveSubsystem.distanceToHub());
    double headingError =
        SwerveMath.normalizeDegrees(
            swerveSubsystem.angleToShootTowards().getDegrees() - swerveSubsystem.getYaw());
    bridge.setAngleToHub(headingError);

    boolean kickerEnabled = bridge.isKickerEnabled();
    if (kickerEnabled) {
      hopperSubsystem.setKickerSpeed(-bridge.getKickerSpeed());
      hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
      hopperSubsystem.setRollerSpeeds(0, INTAKE_SPEED);
    } else if (wasKickerEnabled) {
      hopperSubsystem.setKickerSpeed(0);
      hopperSubsystem.setAgitatorSpeed(0);
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
