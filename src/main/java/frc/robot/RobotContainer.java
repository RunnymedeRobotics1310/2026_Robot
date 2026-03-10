// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Swerve.SUBSYSTEM_CONFIG;
import static frc.robot.Constants.VisionConstants.VISION_CONFIG;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.auto.config.AutoCommandFactory;
import frc.robot.commands.auto.config.AutoCommandRegistrations;
import frc.robot.commands.auto.config.AutoCommandRegistry;
import frc.robot.commands.auto.config.AutoConfigNTBridge;
import frc.robot.commands.shooter.DefaultShooterCommand;
import frc.robot.commands.shooter.ShooterTuneNTBridge;
import frc.robot.commands.swerve.TeleopDriveCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // TODO declare all of the subsystems here
  private final LightingSubsystem lightingSubsystem = new LightingSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(SUBSYSTEM_CONFIG);
  private final LimelightVisionSubsystem visionSubsystem =
      new LimelightVisionSubsystem(VISION_CONFIG, swerveSubsystem);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final AutoCommandRegistry autoCommandRegistry;
  private final AutoCommandFactory autoCommandFactory;
  private final OperatorInput operatorInput;
  private final AutoConfigNTBridge autoConfigNTBridge = new AutoConfigNTBridge();
  private final ShooterTuneNTBridge shooterTuneNTBridge = new ShooterTuneNTBridge();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  {
    // Initialize auto command registry and factory
    autoCommandRegistry = new AutoCommandRegistry();
    AutoCommandRegistrations.registerAll(autoCommandRegistry);

    AutoCommandRegistry.SubsystemRegistry subsystemRegistry =
        new AutoCommandRegistry.SubsystemRegistry();
    subsystemRegistry.register(SwerveSubsystem.class, swerveSubsystem);
    subsystemRegistry.register(ShooterSubsystem.class, shooterSubsystem);
    subsystemRegistry.register(IntakeSubsystem.class, intakeSubsystem);
    subsystemRegistry.register(LimelightVisionSubsystem.class, visionSubsystem);

    autoCommandFactory =
        new AutoCommandFactory(autoCommandRegistry, subsystemRegistry, swerveSubsystem);
    operatorInput =
        new OperatorInput(
            swerveSubsystem,
            shooterSubsystem,
            intakeSubsystem,
            visionSubsystem,
            autoCommandFactory);
  }

  public RobotContainer() {

    // TODO set the default commands for any subsystems
    // NOTE default commands will run when no other command is running
    // and typically take the operator input as the first parameter.

    swerveSubsystem.setDefaultCommand(
        new TeleopDriveCommand(swerveSubsystem, visionSubsystem, operatorInput));

    intakeSubsystem.setDefaultCommand(
        new IntakeCommand(intakeSubsystem, operatorInput, shooterTuneNTBridge));

    shooterSubsystem.setDefaultCommand(
        new DefaultShooterCommand(
            shooterSubsystem, swerveSubsystem, operatorInput, shooterTuneNTBridge));

    // Configure the trigger bindings
    // TODO pass all subsystems to the configure routine
    operatorInput.configureButtonBindings(
        swerveSubsystem, shooterSubsystem, intakeSubsystem, visionSubsystem);
    operatorInput.initAutoSelectors();

    autoConfigNTBridge.setOnConfigsChanged(() -> operatorInput.refreshCustomAutoChooser());
    autoConfigNTBridge.publishCommandMetadata(autoCommandRegistry.getMetadataJson());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return operatorInput.getAutonomousCommand();
  }

  public AutoConfigNTBridge getAutoConfigNTBridge() {
    return autoConfigNTBridge;
  }
}
