// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final LightingSubsystem lightingSubsystem;

  private final ShooterSubsystem shooterSubsystem;

  private final OperatorInput operatorInput;

  private Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, LightingSubsystem lightingSubsystem,
      OperatorInput operatorInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.lightingSubsystem = lightingSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.operatorInput = operatorInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    shooterSubsystem.shooterMotor.set(0.60);
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorInput.getDriverController().getPOV() == 0) {
      shooterSubsystem.kickerMotor.set(0.5);
    } else {
      shooterSubsystem.kickerMotor.set(0.0);
    }

    if (operatorInput.getDriverController().getYButtonPressed() == true) {
      shooterSubsystem.cycleAutoAim();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    shooterSubsystem.shooterMotor.set(0.0);
    shooterSubsystem.kickerMotor.set(0.0);
    timer.stop();
  }
}
