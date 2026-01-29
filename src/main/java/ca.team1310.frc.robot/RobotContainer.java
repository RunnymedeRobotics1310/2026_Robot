// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.team1310.frc.robot;

import ca.team1310.frc.robot.operator.OperatorInput;
import ca.team1310.frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  private final OperatorInput operatorInput =
      new OperatorInput(
          Constants.OiConstants.DRIVER_CONTROLLER_PORT,
          Constants.OiConstants.CONTROLLER_DEADBAND,
          exampleSubsystem);

  //
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize all Subsystem default commands
    //        swerveDriveSubsystem.setDefaultCommand(
    //            new TeleopDriveCommand(
    //                swerveDriveSubsystem,
    //                visionSubsystem,
    //                operatorInput
    //            )
    //        );

    // Configure the trigger bindings
    operatorInput.configureButtonBindings(exampleSubsystem);
  }

  public Command getAutonomousCommand() {
    return null;
    //        return operatorInput.getAutonomousCommand();
  }
}
