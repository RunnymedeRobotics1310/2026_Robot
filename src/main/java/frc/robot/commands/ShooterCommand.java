// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
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

  // Variables for shooter speed control
  public int currentShootMotorRpm = 0;
  private final int maxShooterSpeedRpm = ShooterConstants.maxShooterSpeedRpm;
  private final float Kp = ShooterConstants.Kp; // proportional gain constant for pid controller
  private final float calcSlope = ShooterConstants.calcSlope; // slope for shooting speed calculation
  private final float calcYIntercept = ShooterConstants.calcYIntercept; // y-intercept for shooting speed calculation
  public int calculatedShooterRpm = 0; // Calculated shooter rpm based on distance to hub

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

  public double calculateShootingSpeed(double distanceMeters) {
    double calculatedSpeed = (double) ((distanceMeters * calcSlope) + calcYIntercept);
    if (distanceMeters < 10.0) {
      if (calculatedSpeed <= 1.0) {
        return (calculatedSpeed);
      } else
        return 1.0;
    } else
      return 1.0;
  }

  public void speedPidControl(int setPoint, SparkFlex motor) {
    double currentSpeed = motor.getEncoder().getVelocity();
    double error = (setPoint - currentSpeed) / maxShooterSpeedRpm; // Normalize error
    motor.set((setPoint / maxShooterSpeedRpm) + (error * Kp));
  }

  public void shoot(double distance) {
    calculatedShooterRpm = (int) calculateShootingSpeed(distance);
    speedPidControl(calculatedShooterRpm, shooterSubsystem.shooterMotor);
    if (shooterSubsystem.shooterMotor.getEncoder().getVelocity() >= (calculatedShooterRpm - 35)) {
      shooterSubsystem.kickerMotor.set(0.7);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentShootMotorRpm = (int) shooterSubsystem.shooterMotor.getEncoder().getVelocity();

    double distanceToHub = swerveSubsystem.distanceToHub();

    if (operatorInput.getDriverController().getPOV() == 0) {
      shooterSubsystem.kickerMotor.set(0.7);
    }

    if (operatorInput.getDriverController().getYButtonPressed() == true) {
      shoot(distanceToHub);
    } else
      shooterSubsystem.shooterMotor.set(0.0);

    if (operatorInput.getDriverController().getPOV() != 0 &&
        shooterSubsystem.shooterMotor.getEncoder().getVelocity() < (calculatedShooterRpm - 35)) {
      shooterSubsystem.kickerMotor.set(0.0);
    }

    SmartDashboard.putNumber("Shooter Motor RPM", currentShootMotorRpm);
    SmartDashboard.putNumber("Calculated Shooter RPM", calculatedShooterRpm);

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
