// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

    climbMotor = new SparkMax(50, MotorType.kBrushless);
    secondaryClimbMotor = new SparkMax(51, MotorType.kBrushless);
    pirvate double climbMotorSpeed = 0;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO Update the smartdashboard

    climbMotor.getEncoder().getPosition();
    if (climbMotor.getEncoder().getPosition() < 0) {
    System.out.println("You are at the bottom");
    if (climbMotorSpeed < 0) {
        climbMotorSpeed = 0;
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
public void setClimbMotor(double speed) {
    climbMotorSpeed = speed;

    

}
