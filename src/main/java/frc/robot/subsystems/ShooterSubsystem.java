package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex primaryShooterMotor =
      new SparkFlex(SHOOTER_PRIMARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final SparkFlex secondaryShooterMotor =
      new SparkFlex(SHOOTER_SECONDARY_MOTOR_CAN_ID, SparkFlex.MotorType.kBrushless);
  private final PWMSparkMax kickerMotor = new PWMSparkMax(KICKER_MOTOR_PWM_PORT);
  private final Servo hoodServo = new Servo(HOOD_PWM_PORT);
  private final PWMSparkMax agitatorMotor = new PWMSparkMax(AGITATOR_PWM_PORT);

  private double targetShooterVelocity;
  private double iError = 0;

  private final IntakeSubsystem intake;


  /** Creates The Shooter Subsystem. */
  public ShooterSubsystem(IntakeSubsystem intake) {
    secondaryShooterMotor.configure(
        new SparkFlexConfig().follow(primaryShooterMotor, true),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    this.intake = intake;
  }

  @Override
  public void periodic() {
    Telemetry.shooter.currentShooterRPM = getShooterVelocity();

    updateShooterSpeed();
  }

  public double getShooterVelocity() {
    return primaryShooterMotor.getEncoder().getVelocity();
  }

  public void setShooterVelocity(double target) {
    if (target != targetShooterVelocity) {
      Telemetry.shooter.targetShooterRPM = target;
      targetShooterVelocity = target;
      iError = 0;
    }
//    updateShooterSpeed();
  }

  public void updateShooterSpeed() {
    double currentSpeed = getShooterVelocity();
    double error = (targetShooterVelocity - currentSpeed); // Normalize error
    if (Math.abs(error) > I_ZONE) {
      iError = 0;
    } else {
      iError += error;
      iError = Math.min(iError, (1-error*KP)/KI);
    }

    double pidOutput = (targetShooterVelocity * KFF) + (error * KP) + (iError * KI);
    if (targetShooterVelocity == 0) setShooterSpeed(0);
    else setShooterSpeed(pidOutput);
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
    intake.setRollerSpeeds(0, INTAKE_SPEED);
    Telemetry.shooter.agitatorSpeed = speed;
  }

  /**
   * @param value a value between 0.0 and 1.0
   */
  public void setHood(double value) {
    Telemetry.shooter.hoodAngle = value;
    hoodServo.set(1 - value);
  }

  public double calculateShootingAngle(double distanceMeters) {
    if (distanceMeters <= 5.0 && distanceMeters > 2.0) {
      return 64;
    } else return 78;
  }

  public void stop() {
    setShooterVelocity(0);
    primaryShooterMotor.stopMotor();
    kickerMotor.stopMotor();
    agitatorMotor.stopMotor();
  }
}
