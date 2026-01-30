// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightingSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final OperatorInput operatorInput = new OperatorInput();

  // TODO declare all of the subsystems here
  private final LightingSubsystem lightingSubsystem = new LightingSubsystem();
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem(lightingSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // TODO set the default commands for any subsystems
    // NOTE default commands will run when no other command is running
    // and typically take the operator input as the first parameter.
    exampleSubsystem.setDefaultCommand(new ExampleCommand(exampleSubsystem));

    // Configure the trigger bindings
    // TODO pass all subsystems to the configure routine
    operatorInput.configureButtonBindings(
        lightingSubsystem, exampleSubsystem);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

}
