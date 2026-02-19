// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax primaryClimbMotor = new SparkMax(50, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax secondaryClimbMotor = new SparkMax(51, SparkLowLevel.MotorType.kBrushless);

    private final SparkMax leftHookMotor = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax rightHookMotor = new SparkMax(53, SparkLowLevel.MotorType.kBrushless);

    private double climbMotorSpeed = 0;
    private double leftHookSetpoint = 0;
    private double rightHookSetpoint = 0;


  public ClimbSubsystem() {
  }

  @Override
  public void periodic() {

      // climb safety
    if (primaryClimbMotor.getEncoder().getPosition() < 0) {
        System.out.println("You are at the bottom");
        if (climbMotorSpeed < 0) {
            climbMotorSpeed = 0;
        }
    }
    else if (primaryClimbMotor.getEncoder().getPosition() > MAX_CLIMB_POSITION) {
        System.out.println("You are at the top");
        if (climbMotorSpeed > 0) {
            climbMotorSpeed = 0;
        }
    }
    primaryClimbMotor.set(climbMotorSpeed);
    secondaryClimbMotor.set(climbMotorSpeed);

      // left hook
      double leftDelta = leftHookSetpoint - getLeftHookEncoder();
      if (Math.abs(leftDelta) <= HOOK_TOLERANCE) {
          leftHookMotor.set(0);
      } else if (leftDelta > 0) leftHookMotor.set(-0.1);
      else leftHookMotor.set(0.1);

      // right hook
      double rightDelta = rightHookSetpoint - getRightHookEncoder();
      if (Math.abs(rightDelta) <= HOOK_TOLERANCE) {
          rightHookMotor.set(0);
      } else if (leftDelta > 0) rightHookMotor.set(-0.1);
      else rightHookMotor.set(0.1);

  }

    public void setClimbSpeed(double speed) {
        climbMotorSpeed = speed;
    }

    public void setLeftHookPosition(double position) {
        leftHookSetpoint = position;
    }
    public double getLeftHookEncoder() {
      return leftHookMotor.getEncoder().getPosition();
    }
    public void setRightHookPosition(double position) {
        rightHookSetpoint = position;
    }
    public double getRightHookEncoder() {
        return rightHookMotor.getEncoder().getPosition();
    }

    public void extendLeftHook(){
      setLeftHookPosition(HOOK_EXTENDED_POSITION);
    }
    public void retractLeftHook(){
      setLeftHookPosition(0);
    }
    public void extendRightHook(){
      setRightHookPosition(HOOK_EXTENDED_POSITION);
    }
    public void retractRightHook(){
      setRightHookPosition(0);
    }
    

}
