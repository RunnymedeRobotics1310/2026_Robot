// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax climbMotor =
      new SparkMax(CLIMB_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  //    private final DigitalInput climbLowerLimit = new DigitalInput(CLIMB_LOWER_LIMIT_PORT);

  private double climbMotorSpeed = 0;

  public ClimbSubsystem() {}

  @Override
  public void periodic() {

    // climb safety
    // lower limit
    if (
    /*climbLowerLimit.get()*/ false) {
      // zero encoder
      //      climbMotor.getEncoder().setPosition(0);
      if (climbMotorSpeed < 0) {
        //        climbMotorSpeed = 0;
      }
      // upper limit
    } else if (climbMotor.getEncoder().getPosition() > MAX_CLIMB_POSITION) {
      if (climbMotorSpeed > 0) {
        //        climbMotorSpeed =climbMotorSpeed 0;
      }
      // lower slow zone
    } else if (climbMotor.getEncoder().getPosition() < CLIMB_SLOW_ZONE) {
      if (climbMotorSpeed < -CLIMB_SLOW_ZONE_SPEED) {
        //        climbMotorSpeed = -CLIMB_SLOW_ZONE_SPEED;
      }
      // upper slow zone
    } else if (climbMotor.getEncoder().getPosition() > MAX_CLIMB_POSITION - CLIMB_SLOW_ZONE) {
      if (climbMotorSpeed > CLIMB_SLOW_ZONE_SPEED) {
        //        climbMotorSpeed = CLIMB_SLOW_ZONE_SPEED;
      }
    }
    climbMotor.set(climbMotorSpeed);
  }

  public void setClimbSpeed(double speed) {
    climbMotorSpeed = speed;
  }

  public double getPos() {
    return climbMotor.getEncoder().getPosition();
    //    return 0;
  }

  public void stop() {
    setClimbSpeed(0);
    climbMotor.stopMotor();
  }
}
