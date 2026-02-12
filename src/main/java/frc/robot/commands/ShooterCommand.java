// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.SLOPE_VALUE;
import static frc.robot.Constants.ShooterConstants.Y_INT;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double testShooterSpeed;

  private int lastPov = -1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem vision,
      LightingSubsystem lightingSubsystem,
      OperatorInput operatorInput, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.lightingSubsystem = lightingSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.vision = vision;
    this.operatorInput = operatorInput;
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.reset();
    // shooterSubsystem.shooterMotor.set(1); // Use for testing

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // int currentPOV = operatorInput.getDriverController().getPOV();
    double distance = swerveSubsystem.distanceToHub();
    // shooting(distance);
    SmartDashboard.putNumber("1310/shooter/distanceToHub", distance);
    log("Speed: " + shooterSubsystem.getShooterVelocity());
    if (!timer.isRunning()) {
      timer.start();
    }
    shooting(distance);

    // if (currentPOV == 0 && lastPov == -1) {
    // testShooterSpeed = Math.min(testShooterSpeed + 20, MAX_SHOOTER_RPM);
    // SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    // }

    // if (currentPOV == 180 && lastPov == -1) {
    // testShooterSpeed = Math.max(testShooterSpeed - 20, 0);
    // SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    // }

    // if (operatorInput.getDriverController().getYButton()) {
    // shooterSubsystem.setShooterVelocity(testShooterSpeed);
    // SmartDashboard.putNumber("1310/shooter/currentspeed",
    // shooterSubsystem.getShooterVelocity());
    // } else
    // shooterSubsystem.setShooterSpeed(0.0);

    // lastPov = currentPOV;

    // if (operatorInput.getDriverController().getPOV() == 270) {
    // shooterSubsystem.setKickerSpeed(0.7);
    // } else
    // shooterSubsystem.setKickerSpeed(0.0);

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
    shooterSubsystem.setShooterSpeed(0.0);
    shooterSubsystem.setKickerSpeed(0.0);
    timer.stop();
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    if (distanceMeters < 10.0) {
      shooterSpeed = (distanceMeters * SLOPE_VALUE) + Y_INT;
      // log("Target speed: " + shooterSpeed);
    }
    return shooterSpeed;
  }

  public void shooting(double distance) {
    double shooterSpeed = calculateShootingSpeed(distance);
    shooterSubsystem.setShooterVelocity(shooterSpeed);
    SmartDashboard.putNumber("1310/shooter/targetspeed", shooterSpeed);

    // Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed) < 10

    if (timer.hasElapsed(2.5)) {
      shooterSubsystem.setKickerSpeed(0.7);
    }
    if (timer.hasElapsed(2.8)) {
      shooterSubsystem.setKickerSpeed(0.0);
      timer.reset();
      timer.stop();
    }

  }
}
