// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax primaryClimbMotor = new SparkMax(50, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax secondaryClimbMotor = new SparkMax(51, SparkLowLevel.MotorType.kBrushless);

    private final Solenoid hookSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    private double climbMotorSpeed = 0;


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

  }

    public void setClimbSpeed(double speed) {
        climbMotorSpeed = speed;
    }

    public void deployHook() {
        hookSolenoid.set(true);
    }

}
