package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class HopperSubsystem extends SubsystemBase {

  private final SparkFlex primaryShooterMotor =
      new SparkFlex(SHOOTER_PRIMARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final SparkFlex secondaryShooterMotor =
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

  private double targetShooterVelocity;
  private double iError = 0;
  private double doorSetpoint = 0;

  public HopperSubsystem() {
    secondaryShooterMotor.configure(
        new SparkFlexConfig().follow(primaryShooterMotor, true),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Telemetry.shooter.currentShooterRPM = getShooterVelocity();
    Telemetry.intake.doorSetpoint = doorSetpoint;
    Telemetry.intake.doorAngle = getDoorAngle();
    Telemetry.intake.isDoorClosed = getDoorClosed();

    updateShooterSpeed();
    updateDoorSpeed();

    if (getDoorClosed()) {
      doorMotor.getEncoder().setPosition(0);
    }
  }

  public double getShooterVelocity() {
    return primaryShooterMotor.getEncoder().getVelocity();
  }

  public void setShooterVelocity(double target) {
    Telemetry.shooter.targetShooterRPM = target;
    targetShooterVelocity = target;
    if (Math.abs(target - targetShooterVelocity) > ACCPETED_SHOOTER_ERROR) {
      iError = 0;
    }
  }

  public void setShooterSpeed(double speed) {
    primaryShooterMotor.set(speed);
  }

  public void setKickerSpeed(double speed) {
    Telemetry.shooter.kickerSpeed = speed;
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
    doorMotor.set(doorSpeed);
  }

  public boolean getDoorClosed() {
    return !doorClosedLimit.get();
  }

  public double calculateShootingAngle(double distanceMeters) {
    if (distanceMeters <= 5.0 && distanceMeters > 2.0) {
      return 64;
    } else return 78;
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    if (distanceMeters < MAX_SHOOTING_DISTANCE) {

      if (distanceMeters >= HOOD_SHOOT_DISTANCE) {
        double aVal = A_VALUE_WITH_HOOD * Math.pow(distanceMeters, 3);
        double bVal = B_VALUE_WITH_HOOD * Math.pow(distanceMeters, 2);
        double cVal = C_VALUE_WITH_HOOD * distanceMeters;
        double dVal = D_VALUE_WITH_HOOD;
        shooterSpeed = (aVal + bVal + cVal + dVal);
      } else {
        double aVal = A_VALUE_NO_HOOD * Math.pow(distanceMeters, 2);
        double bVal = B_VALUE_NO_HOOD * distanceMeters;
        double cVal = C_VALUE_NO_HOOD;
        shooterSpeed = (aVal + bVal + cVal);
      }
    }
    if (shooterSpeed > MAX_SHOOTER_RPM) shooterSpeed = MAX_SHOOTER_RPM;
    else if (shooterSpeed < 0.0) shooterSpeed = 0.0;
    return shooterSpeed;
  }

  public double calculateHoodValule(double distance) {
    double hoodValue = 0;
    if (distance >= HOOD_SHOOT_DISTANCE) {
      double aVal = HOOD_A_VALUE * Math.pow(distance, 3);
      double bVal = HOOD_B_VALUE * Math.pow(distance, 2);
      double cVal = HOOD_C_VALUE * distance;
      double dVal = HOOD_D_VALUE;
      hoodValue = (aVal + bVal + cVal + dVal);
    }
    if (hoodValue > 1.0) hoodValue = 1.0;
    else if (hoodValue < 0.0) hoodValue = 0.0;

    return hoodValue;
  }

  private void updateShooterSpeed() {
    double currentSpeed = getShooterVelocity();
    double error = (targetShooterVelocity - currentSpeed); // Normalize error
    if (Math.abs(error) > I_ZONE) {
      iError = 0;
    } else {
      iError += error;
      iError = Math.min(iError, (1 - error * KP) / KI);
    }

    double pidOutput = (targetShooterVelocity * KFF) + (error * KP) + (iError * KI);
    if (targetShooterVelocity == 0) setShooterSpeed(0);
    else setShooterSpeed(pidOutput);
  }

  private void updateDoorSpeed() {
    if (doorSetpoint == -1) return;
    double error = doorSetpoint - getDoorAngle();

    setDoorSpeed(error * DOOR_KP);

    if (doorSetpoint == 0 && !getDoorClosed()) {
      setDoorSpeed(-0.1);
    }
  }

  public void stop() {
    setShooterVelocity(0);
    setDoorSetpoint(0);

    primaryShooterMotor.stopMotor();
    kickerMotor.stopMotor();
    agitatorMotor.stopMotor();
    setRollerSpeeds(0, 0);
    setDoorSpeed(0);
  }
}
