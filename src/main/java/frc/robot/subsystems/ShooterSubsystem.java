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

  public float shooterAngleDegrees = 0;
  public int calculatedShooterMotorRpm = 0;

  public float hubAngle = 0;
  public float hubAngleOffset = 0;

  public boolean autoAiming = false;

  /** Creates The Shooter Subsystem. */
  public ShooterSubsystem(LightingSubsystem lightingSubsystem) {
    this.lightingSubsystem = lightingSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double calculateShootingAngle(float distanceMeters) {
    return 28 * (Math.pow(Math.E, (-0.231 * distanceMeters)) + 52);
  }

  // To do: Edit this method to return the actual shooting speed value

  public double calculateShootingSpeed(double distanceMeters) {
    if (distanceMeters < 1.0) {
      return (distanceMeters * 0.08) + 0.2;
    } else {
      return 1.0;
    }
  }

  public void autoAim() {
    if (Math.abs(hubAngleOffset) > 2.0) {
      if (hubAngleOffset >= 180.0) {
        /*
         * Turn right
         * Untill the hubAngle is within 3 degrees of the current robot angle
         */

      } else {
        /*
         * Turn left
         * Untill the hubAngle is within 3 degrees of the current robot angle
         */
      }
    }
  }

  public void cycleAutoAim() {
    autoAiming = !autoAiming;
  }

}
