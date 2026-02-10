// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.KP;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTER_RPM;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final LightingSubsystem lightingSubsystem;

  public final SparkFlex shooterMotor = new SparkFlex(30, SparkFlex.MotorType.kBrushless);
  public final SparkFlex kickerMotor = new SparkFlex(33, SparkFlex.MotorType.kBrushless);

  public float hubDistanceMeters = 0;
  public float shooterAngleDegrees = 0;
  public int shooterSpeedRpm = 0;
  public int kickerSpeedRpm = 0;

  public float hubAngle = 0;
  public float hubAngleOffset = 0;
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
    return shooterMotor.getEncoder().getVelocity();
  }

  public double getKickerVelocity() {
    return kickerMotor.getEncoder().getVelocity();
  }

  public void setKickerVelocity(double setPoint) {
    double currentSpeed = getKickerVelocity();
    double error = (setPoint - currentSpeed) / MAX_SHOOTER_RPM; // Normalize error
    kickerMotor.set((setPoint / MAX_SHOOTER_RPM) + (error * KP));
  }

  public void setShooterVelocity(double setPoint) {
    targetMotorVelocity = setPoint;
    double currentSpeed = getShooterVelocity();
    double error = (setPoint - currentSpeed) / MAX_SHOOTER_RPM; // Normalize error
    shooterMotor.set((setPoint / MAX_SHOOTER_RPM) + (error * KP));
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setKickerSpeed(double speed) {
    kickerMotor.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double calculateShootingAngle(float distanceMeters) {
    return 28 * (Math.pow(Math.E, (-0.231 * distanceMeters)) + 52);
  }

  public void stop() {
    shooterMotor.stopMotor();
    kickerMotor.stopMotor();
  }
}
