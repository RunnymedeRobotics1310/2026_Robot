// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.KP;
import static frc.robot.Constants.ShooterConstants.IS_HOPPER_ATTACHED;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTER_RPM;

import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final LightingSubsystem lightingSubsystem;

  private final SparkFlex shooterMotor =
          IS_HOPPER_ATTACHED ? new SparkFlex(30, SparkFlex.MotorType.kBrushless) : null;
  private final SparkMax kickerMotor =
          IS_HOPPER_ATTACHED ? new SparkMax(33, SparkFlex.MotorType.kBrushless) : null;
  private final Servo hoodServo = new Servo(8);

  public double hubDistanceMeters = 0;
  public double shooterAngleDegrees = 0;
  public int shooterSpeedRpm = 0;
  public int kickerSpeedRpm = 0;

  public double hubAngle = 0;
  public double hubAngleOffset = 0;
  private double targetMotorVelocity;

  public boolean autoAiming = false;

  /** Creates The Shooter Subsystem. */
  public ShooterSubsystem(LightingSubsystem lightingSubsystem) {
    this.lightingSubsystem = lightingSubsystem;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("1310/shooter/currentmotorvelocity", getShooterVelocity());
    SmartDashboard.putNumber("1310/shooter/targetmotorvelocity", targetMotorVelocity);

  }

  public double getShooterVelocity() {
    if (IS_HOPPER_ATTACHED) return shooterMotor.getEncoder().getVelocity();
    else return 0;
  }

  public double getKickerVelocity() {
    if (IS_HOPPER_ATTACHED) return kickerMotor.getEncoder().getVelocity();
    else return 0;
  }

  public void setKickerVelocity(double setPoint) {
    if (IS_HOPPER_ATTACHED) {
      double currentSpeed = getKickerVelocity();
      double error = (setPoint - currentSpeed) / MAX_SHOOTER_RPM; // Normalize error
      kickerMotor.set((setPoint / MAX_SHOOTER_RPM) + (error * KP));
    } else System.out.println("SETTING KICKER VELOCITY TO: " + setPoint);
  }

  public void setShooterVelocity(double setPoint) {
    if (IS_HOPPER_ATTACHED) {
      targetMotorVelocity = setPoint;
      double currentSpeed = getShooterVelocity();
      double error = (setPoint - currentSpeed) / MAX_SHOOTER_RPM; // Normalize error
      shooterMotor.set((setPoint / MAX_SHOOTER_RPM) + (error * KP));
    } else System.out.println("SETTING SHOOTER VELOCITY TO: " + setPoint);
  }

  public void setShooterSpeed(double speed) {
    if (IS_HOPPER_ATTACHED) shooterMotor.set(speed);
    else System.out.println("SETTING SHOOTER SPEED TO: " + speed);

}

  public void setKickerSpeed(double speed) {
    if (IS_HOPPER_ATTACHED) kickerMotor.set(speed);
    else System.out.println("SETTING KICKER SPEED TO: " + speed);

}

  /**
   * @param value a value between 0.0 and 1.0
   */
  public void setHood(double value) {
    hoodServo.set(value);
    SmartDashboard.putNumber("1310/shooter/hoodAngle", value);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double calculateShootingAngle(double distanceMeters) {
    return 28 * (Math.pow(Math.E, (-0.231 * distanceMeters)) + 52);
  }

  public void stop() {
    if (IS_HOPPER_ATTACHED) {
      shooterMotor.stopMotor();
      kickerMotor.stopMotor();
    } else System.out.println("STOPPING SHOOTER");
  }
}
