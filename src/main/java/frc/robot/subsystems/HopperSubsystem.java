package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class HopperSubsystem extends SubsystemBase {

  private final SparkFlex primaryShooterMotor =
      new SparkFlex(SHOOTER_PRIMARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final SparkFlex secondaryShooterMotor =
      new SparkFlex(SHOOTER_SECONDARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final PWMSparkMax kickerMotor = new PWMSparkMax(KICKER_MOTOR_PWM_PORT);
  private final PWMSparkMax agitatorMotor = new PWMSparkMax(AGITATOR_PWM_PORT);
  private final Servo hoodServo = new Servo(HOOD_PWM_PORT);

  private final PWMSparkMax bottomRollerMotor = new PWMSparkMax(BOTTOM_ROLLER_PWM_PORT);
  private final PWMSparkMax topRollerMotor = new PWMSparkMax(TOP_ROLLER_PWM_PORT);
  private final SparkMax doorMotor = new SparkMax(DOOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final DigitalInput doorClosedLimit = new DigitalInput(DOOR_CLOSED_LIMIT_DIO_PORT);
  //  private final DigitalInput beamBreak = new DigitalInput(-1);

  private final PIDController shooterController = new PIDController(KP, KI, KD, 20.0 / 1000);

  private double targetShooterVelocity;
  private double doorSetpoint = 0;

  /** Creates The Shooter Subsystem. */
  public HopperSubsystem() {
    secondaryShooterMotor.configure(
        new SparkFlexConfig().follow(primaryShooterMotor, true),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Telemetry.shooter.targetShooterRPM = targetShooterVelocity;
    Telemetry.shooter.currentShooterRPM = getShooterVelocity();

    Telemetry.intake.isHopperFull = isBeamBroken();
    Telemetry.intake.doorSetpoint = doorSetpoint;
    Telemetry.intake.doorAngle = getDoorAngle();
    Telemetry.intake.isDoorClosed = getDoorClosed();

    updateDoorSpeed();

    if (getDoorClosed()) {
      doorMotor.getEncoder().setPosition(0);
    }
  }

  public void setShooterVelocity(double target) {
    targetShooterVelocity = target;
    primaryShooterMotor.set((target * KFF) + shooterController.calculate(getShooterVelocity(), target));
  }

  public void setShooterSpeed(double speed) {
    primaryShooterMotor.set(speed);
  }

  public double getShooterVelocity() {
    return primaryShooterMotor.getEncoder().getVelocity();
  }

  public void setKickerSpeed(double speed) {
    kickerMotor.set(speed);
    Telemetry.shooter.kickerSpeed = speed;
  }

  public void setAgitatorSpeed(double speed) {
    Telemetry.shooter.agitatorSpeed = speed;
    agitatorMotor.set(speed);
  }

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

  public void setDoorSpeed(double doorSpeed) {
    doorMotor.set(doorSpeed);
  }

  public void setDoorSetpoint(double setpoint) {
    doorSetpoint = setpoint;
  }

  public double getDoorAngle() {
    return doorMotor.getEncoder().getPosition() * DOOR_ENCODERS_TO_DEGREES;
  }

  public boolean getDoorClosed() {
    return !doorClosedLimit.get();
  }

  public boolean isBeamBroken() {
    // return !beamBreak.get(); // Assuming the sensor returns false when the beam is broken
    return false;
  }

  private void updateDoorSpeed() {
    double error = doorSetpoint - getDoorAngle();

    setDoorSpeed(error * DOOR_KP);

    if (doorSetpoint == 0 && !getDoorClosed()) {
      setDoorSpeed(-0.1);
    }
  }

  public void stop() {
    primaryShooterMotor.stopMotor();
    kickerMotor.stopMotor();
    agitatorMotor.stopMotor();
    setRollerSpeeds(0, 0);
    setDoorSpeed(0);
  }
}
