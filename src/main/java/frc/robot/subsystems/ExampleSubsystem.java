// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  LightingSubsystem lightingSubsystem;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem(LightingSubsystem lightingSubsystem) {
    this.lightingSubsystem = lightingSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO Update the smartdashboard
    // TODO Update the lights
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
