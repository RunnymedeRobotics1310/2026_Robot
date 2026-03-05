package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.AGITATOR_PWM_PORT;
import static frc.robot.Constants.ShooterConstants.HOOD_PWM_PORT;
import static frc.robot.Constants.ShooterConstants.KFF;
import static frc.robot.Constants.ShooterConstants.KICKER_MOTOR_PWM_PORT;
import static frc.robot.Constants.ShooterConstants.KP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_PRIMARY_MOTOR_CAN_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SECONDARY_MOTOR_CAN_ID;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex primaryShooterMotor = new SparkFlex(SHOOTER_PRIMARY_MOTOR_CAN_ID,
      SparkFlex.MotorType.kBrushless);
  private final SparkFlex secondaryShooterMotor = new SparkFlex(SHOOTER_SECONDARY_MOTOR_CAN_ID,
      SparkFlex.MotorType.kBrushless);
  private final PWMSparkMax kickerMotor = new PWMSparkMax(KICKER_MOTOR_PWM_PORT);
  private final Servo hoodServo = new Servo(HOOD_PWM_PORT);
  private final PWMSparkMax agitatorMotor = new PWMSparkMax(AGITATOR_PWM_PORT);

  private double targetShooterVelocity;

  /** Creates The Shooter Subsystem. */
  public ShooterSubsystem() {
    secondaryShooterMotor.configure(
        new SparkFlexConfig().follow(primaryShooterMotor, true),
        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Telemetry.shooter.targetShooterRPM = targetShooterVelocity;
    Telemetry.shooter.currentShooterRPM = getShooterVelocity();

  }

  public double getShooterVelocity() {
    return primaryShooterMotor.getEncoder().getVelocity();
  }

  public void setShooterVelocity(double target) {
    targetShooterVelocity = target;
    double currentSpeed = getShooterVelocity();
    double error = (target - currentSpeed); // Normalize error
    primaryShooterMotor.set((target * KFF) + (error * KP));
  }

  public void setShooterSpeed(double speed) {
    primaryShooterMotor.set(speed);
  }

  public void setKickerSpeed(double speed) {
    kickerMotor.set(speed);
    Telemetry.shooter.kickerSpeed = speed;
  }

  public void setAgitatorSpeed(double speed) {
    agitatorMotor.set(speed);
    Telemetry.shooter.agitatorSpeed = speed;
  }

  /**
   * @param value a value between 0.0 and 1.0
   */
  public void setHood(double value) {
    hoodServo.set(1 - value); // FIXME: Maybe take out the 1 - part later
    Telemetry.shooter.hoodAngle = 1 - value;
  }

  public double calculateShootingAngle(double distanceMeters) {
    if (distanceMeters <= 5.0 && distanceMeters > 2.0) {
      return 64;
    } else
      return 78;
  }

  public void stop() {
    primaryShooterMotor.stopMotor();
    kickerMotor.stopMotor();
    agitatorMotor.stopMotor();
  }

}