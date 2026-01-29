// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.team1310.frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

// import ca.team1310.frc.robot.commands.operator.OperatorInput;
// import ca.team1310.frc.robot.commands.swervedrive.TeleopDriveCommand;
// import ca.team1310.frc.robot.subsystems.swerve.SwerveSubsystem;
// import ca.team1310.frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem(
  //        Constants.Swerve.SUBSYSTEM_CONFIG
  //    );
  //    private final LimelightVisionSubsystem visionSubsystem =
  //        new LimelightVisionSubsystem(
  //            Constants.VisionConstants.VISION_CONFIG,
  //            swerveDriveSubsystem
  //        );
  //    private final OperatorInput operatorInput = new OperatorInput(
  //        OiConstants.DRIVER_CONTROLLER_PORT,
  //        OiConstants.CONTROLLER_DEADBAND,
  //        swerveDriveSubsystem,
  //        visionSubsystem
  //    );
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
    //
    // Configure the trigger bindings
    //        operatorInput.configureButtonBindings(
    //            swerveDriveSubsystem,
    //            visionSubsystem
    //        );
    //
    //        operatorInput.initAutoSelectors();
  }

  public Command getAutonomousCommand() {
    return null;
    //        return operatorInput.getAutonomousCommand();
  }
}
