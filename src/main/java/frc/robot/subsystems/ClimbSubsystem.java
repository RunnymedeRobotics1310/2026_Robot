// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax climbMotor =
      new SparkMax(CLIMB_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final DigitalInput climbLowerLimit = new DigitalInput(CLIMB_LOWER_LIMIT_DIO_PORT);

  private double climbMotorSpeed = 0;

  public ClimbSubsystem() {}

  @Override
  public void periodic() {

    /* ----- CLIMB SAFETY ----- */
    // lower limit
    if (isClimbDown()) {
      // zero encoder
      zeroEncoder();
      if (climbMotorSpeed < 0) {
        climbMotorSpeed = 0;
      }
      // upper limit
    } else if (getClimbPosition() > MAX_CLIMB_POSITION) {
      if (climbMotorSpeed > 0) {
        climbMotorSpeed = 0;
      }
      // upper slow zone
    } else if (getClimbPosition() > MAX_CLIMB_POSITION - CLIMB_SLOW_ZONE) {
      if (climbMotorSpeed > CLIMB_SLOW_ZONE_SPEED) {
        climbMotorSpeed = CLIMB_SLOW_ZONE_SPEED;
      }
    }
    climbMotor.set(climbMotorSpeed);

    Telemetry.climb.climbSpeed = climbMotorSpeed;
    Telemetry.climb.climbPosition = getClimbPosition();
    Telemetry.climb.climbDown = isClimbDown();
  }

  public void setClimbSpeed(double speed) {
    climbMotorSpeed = speed;
  }

  public void zeroEncoder() {
    climbMotor.getEncoder().setPosition(0);
  }

  public double getClimbPosition() {
    return climbMotor.getEncoder().getPosition();
  }

  public boolean isClimbDown() {
    return !climbLowerLimit.get();
  }

  public void stop() {
    setClimbSpeed(0);
    climbMotor.stopMotor();
  }
}
