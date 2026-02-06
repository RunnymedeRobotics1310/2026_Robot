// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final LightingSubsystem lightingSubsystem;

  private final ShooterSubsystem shooterSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final LimelightVisionSubsystem vision;

  private final OperatorInput operatorInput;

  private Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem vision,
      LightingSubsystem lightingSubsystem, SwerveSubsystem swerveSubsystem,
      OperatorInput operatorInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.lightingSubsystem = lightingSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.vision = vision;
    this.operatorInput = operatorInput;
  }

  public void shoot(double distance) {
    shooterSubsystem.calculatedShooterMotorRpm = (int) shooterSubsystem.calculateShootingSpeed(distance);
    speedPidControl(shooterSubsystem.calculatedShooterMotorRpm, shooterSubsystem.kickerMotor);
    if (shooterSubsystem.shooterMotor.getEncoder().getVelocity() >= (shooterSubsystem.calculatedShooterMotorRpm - 25)) {
      shooterSubsystem.kickerMotor.set(0.7);
    } else {
      shooterSubsystem.kickerMotor.set(0.0);
    }
  }

  public void speedPidControl(int setPoint, SparkFlex motor) {
    double currentSpeed = motor.getEncoder().getVelocity();
    double error = (setPoint - currentSpeed) / Constants.ShooterConstants.maxShooterSpeedRpm; // Normalize error
    motor.set((setPoint / Constants.ShooterConstants.maxShooterSpeedRpm) + (error * Constants.ShooterConstants.Kp));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    shooterSubsystem.shooterMotor.set(0.7);
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distanceToHub = swerveSubsystem.distanceToHub();

    if (operatorInput.getDriverController().getPOV() == 0) {
      shooterSubsystem.kickerMotor.set(0.7);
    } else {
      shooterSubsystem.kickerMotor.set(0.0);
    }

    if (operatorInput.getDriverController().getYButtonPressed() == true) {
      shoot(distanceToHub);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return operatorInput.getDriverController().getXButton();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    shooterSubsystem.shooterMotor.set(0.0);
    shooterSubsystem.kickerMotor.set(0.0);
    timer.stop();
    timer.reset();
  }
}
