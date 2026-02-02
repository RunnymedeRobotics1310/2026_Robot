// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final LightingSubsystem lightingSubsystem;

  public final SparkFlex shooterMotor = new SparkFlex(30, SparkFlex.MotorType.kBrushless);
  public final SparkFlex kickerMotor = new SparkFlex(33, SparkFlex.MotorType.kBrushless);

  public float hubDistanceInches = 0;
  public float shooterAngleDegrees = 0;
  public int shooterSpeedRpm = 0;

  /** Creates The Shooter Subsystem. */
  public ShooterSubsystem(LightingSubsystem lightingSubsystem) {
    this.lightingSubsystem = lightingSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO Update the hub distance here ***

    shooterAngleDegrees = Math.round(calculateShootingAngle(hubDistanceInches) * 100.0) / 100.0f;
    shooterSpeedRpm = (int) Math.round(calculateShootingSpeed(hubDistanceInches));

    // TODO: Make the shooter info actually change the shooter angle and spped

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double calculateShootingAngle(float distanceInches) {
    return 20 * (Math.pow(Math.E, -0.000587 * distanceInches) + 48);
  }

  // To do: Edit this method to return the actual shooting speed value

  public double calculateShootingSpeed(float distanceInches) {
    return (distanceInches * 0.01) + 0.2;
  }
}
