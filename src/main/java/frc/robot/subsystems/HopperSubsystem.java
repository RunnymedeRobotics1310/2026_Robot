package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import ca.team1310.swerve.utils.SwerveUtils;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.ShotCounter;
import frc.robot.telemetry.Telemetry;

public class HopperSubsystem extends SubsystemBase {

  private final SparkFlex rightShooterMotor =
      new SparkFlex(SHOOTER_PRIMARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final SparkFlex leftShooterMotor =
      new SparkFlex(SHOOTER_SECONDARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final SparkMax kickerMotor =
      new SparkMax(KICKER_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax agitatorMotor =
      new SparkMax(AGITATOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final Servo hoodServo = new Servo(HOOD_PWM_PORT);

  private final SparkMax bottomRollerMotor =
      new SparkMax(BOTTOM_ROLLER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax topRollerMotor =
      new SparkMax(TOP_ROLLER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax doorMotor = new SparkMax(DOOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

  private final DigitalInput doorClosedLimit = new DigitalInput(DOOR_CLOSED_LIMIT_DIO_PORT);
  private final DigitalInput doorOpenLimit = new DigitalInput(DOOR_OPEN_LIMIT_DIO_PORT);

  private double targetShooterVelocity;
  private double rIError = 0;
  private double lIError = 0;
  private double doorSetpoint = 0;
  private double targetRollerSpeed = 0;

  private final Timer agitatorTimer = new Timer();
  private boolean agitatorState = false;

  private final Timer doorPulseTimer = new Timer();
  private boolean doorPulseState = false;
  private boolean isDoorPulsing = false;

  private ShotCounter shotCounter = new ShotCounter();

  public HopperSubsystem() {
    agitatorTimer.start();
    agitatorTimer.reset();
    doorPulseTimer.start();
    doorPulseTimer.reset();
  }

  @Override
  public void periodic() {
    double leftRpm = getLeftShooterVelocity();
    double rightRpm = getRightShooterVelocity();

    shotCounter.update(targetShooterVelocity, leftRpm, rightRpm);

    Telemetry.shooter.currentRightShooterRPM = rightRpm;
    Telemetry.shooter.currentLeftShooterRPM = leftRpm;
    Telemetry.shooter.kickerSpeed = kickerMotor.getEncoder().getVelocity();
    Telemetry.intake.doorSetpoint = doorSetpoint;
    Telemetry.intake.doorAngle = getDoorAngle();
    Telemetry.intake.isDoorClosed = getDoorClosed();
    Telemetry.intake.isDoorPulsing = isDoorPulsing;

    updateShooterSpeed();
    updateDoorSpeed();

    if (getDoorClosed()) {
      doorMotor.getEncoder().setPosition(0);
    }
  }

  public double getRightShooterVelocity() {
    return rightShooterMotor.getEncoder().getVelocity();
  }

  public double getLeftShooterVelocity() {
    return leftShooterMotor.getEncoder().getVelocity();
  }

  public void setShooterVelocity(double target) {
    Telemetry.shooter.targetShooterRPM = target;
    if (Math.abs(target - targetShooterVelocity) > ACCPETED_SHOOTER_ERROR / 3) {
      rIError = 0;
      lIError = 0;
    }
    targetShooterVelocity = target;
  }

  public void setShooterSpeed(double speed) {
    setRightShooterSpeed(speed);
    setLeftShooterSpeed(speed);
  }

  public void setRightShooterSpeed(double speed) {
    rightShooterMotor.set(speed);
  }

  public void setLeftShooterSpeed(double speed) {
    leftShooterMotor.set(speed);
  }

  public void setKickerSpeed(double speed) {
    Telemetry.shooter.kickerTarget = speed;
    kickerMotor.set(speed);
  }

  public void setAgitatorSpeed(double speed) {
    agitatorMotor.set(speed);
    Telemetry.shooter.agitatorSpeed = speed;
  }

  /**
   * @param value a value between 0.0 and 1.0
   */
  public void setHood(double value) {
    Telemetry.shooter.hoodAngle = value;
    hoodServo.set(1 - value);
  }

  public void setRollerSpeeds(double topRollerSpeed, double bottomRollerSpeed) {
    Telemetry.intake.topRollerSpeed = topRollerSpeed;
    Telemetry.intake.bottomRollerSpeed = bottomRollerSpeed;
    topRollerMotor.set(topRollerSpeed);
    bottomRollerMotor.set(bottomRollerSpeed);
  }

  public void setDoorSetpoint(double setpoint) {
    doorSetpoint = setpoint;
  }

  public double getDoorAngle() {
    return doorMotor.getEncoder().getPosition() * DOOR_ENCODERS_TO_DEGREES;
  }

  public void setDoorSpeed(double doorSpeed) {
    if (getDoorClosed() && doorSpeed < 0) doorSpeed = 0;
    if (getDoorDown() && doorSpeed > 0) doorSpeed = 0;
    doorMotor.set(doorSpeed);
    //    doorMotor.set(0);
  }

  public boolean getDoorClosed() {
    return !doorClosedLimit.get();
  }

  public boolean getDoorDown() {
    return !doorOpenLimit.get();
  }

  public double calculateShootingAngle(double distanceMeters) {
    if (distanceMeters <= 5.0 && distanceMeters > 2.0) {
      return 64;
    } else return 78;
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    double hoodValue = calculateHoodValule(distanceMeters);

    shooterSpeed = 610 * distanceMeters + 3000 - hoodValue * 1000;

    return shooterSpeed;
  }

  public double calculateHoodValule(double distance) {
    double hoodValue = 0;

    hoodValue = SwerveUtils.clamp(0, .3 * distance - .5, 1);

    return hoodValue;
  }

  private void updateShooterSpeed() {
    /* ----- RIGHT SHOOTER ----- */
    double rSpeed = getRightShooterVelocity();
    double rError = (targetShooterVelocity - rSpeed); // Normalize error
    if (Math.abs(rError) > I_ZONE) {
      rIError = 0;
    } else {
      rIError += rError;
      rIError = Math.min(rIError, (1 - rError * KP) / KI);
    }

    /* ----- LEFT SHOOTER ----- */
    double lSpeed = getLeftShooterVelocity();
    double lError = (targetShooterVelocity - lSpeed);
    if (Math.abs(lError) > I_ZONE) {
      lIError = 0;
    } else {
      lIError += lError;
      lIError = Math.min(lIError, (1 - lError * KP) / KI);
    }

    double pidOutputR = (targetShooterVelocity * KFF) + (rError * KP) + (rIError * KI);
    double pidOutputL = (targetShooterVelocity * KFF) + (lError * KP) + (lIError * KI);

    if (targetShooterVelocity == 0) setShooterSpeed(0);
    else {
      setRightShooterSpeed(pidOutputR);
      setLeftShooterSpeed(pidOutputL);
    }
  }

  private void updateDoorSpeed() {
    if (isDoorPulsing) return;
    if (doorSetpoint == -1) return;
    double error = doorSetpoint - getDoorAngle();

    setDoorSpeed(error * DOOR_KP);

    if (doorSetpoint == 0 && !getDoorClosed()) {
      setDoorSpeed(-DOOR_SPEED);
    }
  }

  public void stop() {
    setShooterVelocity(0);
    setDoorSetpoint(0);

    setShooterSpeed(0);
    setKickerSpeed(0);
    setAgitatorSpeed(0);
    setRollerSpeeds(0, 0);
    setDoorSpeed(0);

    isDoorPulsing = false;
  }

  public void pulseAgitator(double period) {
    if (agitatorTimer.hasElapsed(period)) {
      agitatorTimer.reset();
      agitatorState = !agitatorState;
    }

    if (agitatorState) {
      setAgitatorSpeed(AGITATOR_RUNSPEED);
    } else {
      setAgitatorSpeed(0);
    }
  }

  public void pulseDoor(double period) {
    isDoorPulsing = true;
    if (doorPulseTimer.get() >= period) {
      doorPulseTimer.reset();
      doorPulseState = !doorPulseState;
    }

    if (doorPulseState) {
      setDoorSpeed(DOOR_SPEED / 2);
    } else {
      setDoorSpeed(-DOOR_SPEED);
    }
  }

  public void stopDoorPulsing() {
    isDoorPulsing = false;
  }

  public void reverseAgitator(double period) {
    if (agitatorTimer.hasElapsed(period)) {
      agitatorTimer.reset();
      agitatorState = !agitatorState;
    }

    if (agitatorState) {
      setAgitatorSpeed(AGITATOR_RUNSPEED);
    } else {
      setAgitatorSpeed(-AGITATOR_RUNSPEED);
    }
  }

  public boolean isShooterAtSpeed() {

    if (targetShooterVelocity == 0) return false;

    boolean leftAtSpeed =
        Math.abs(targetShooterVelocity - getLeftShooterVelocity()) < ACCPETED_SHOOTER_ERROR;
    boolean rightAtSpeed =
        Math.abs(targetShooterVelocity - getRightShooterVelocity()) < ACCPETED_SHOOTER_ERROR;

    return leftAtSpeed && rightAtSpeed;
  }

  public double getBottomRollerSpeed() {
    return bottomRollerMotor.get();
  }

  public double getTopRollerSpeed() {
    return topRollerMotor.get();
  }

  public int getAutoShotCount() {
    return shotCounter.getAutoShotCount();
  }

  public int getTeleopShotCount() {
    return shotCounter.getTeleopShotCount();
  }

  public int getTotalShotCount() {
    return shotCounter.getTotalShotCount();
  }

  public void resetShotCount() {
    shotCounter.reset();
  }
}
