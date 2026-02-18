// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax climbMotor = new SparkMax(50, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax secondaryClimbMotor = new SparkMax(51, SparkLowLevel.MotorType.kBrushless);
    private double climbMotorSpeed = 0;


  public ClimbSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO Update the smartdashboard

    if (climbMotor.getEncoder().getPosition() < 0) {
        System.out.println("You are at the bottom");
        if (climbMotorSpeed < 0) {
            climbMotorSpeed = 0;
        }
    }
    else if (climbMotor.getEncoder().getPosition() > 100) {
        System.out.println("You are at the top");
        if (climbMotorSpeed > 0) {
            climbMotorSpeed = 0;
        }
    }
    climbMotor.set(climbMotorSpeed);
    secondaryClimbMotor.set(climbMotorSpeed);
  }

    public void setClimbMotor(double speed) {
        climbMotorSpeed = speed;
    }
    

}
