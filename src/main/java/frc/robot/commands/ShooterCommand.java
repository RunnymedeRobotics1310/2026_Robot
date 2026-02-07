// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.spark.SparkFlex;
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

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem vision, LightingSubsystem lightingSubsystem,
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
//    shooterSubsystem.shooterMotor.set(1); // Use for testing
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = swerveSubsystem.distanceToHub();
    SmartDashboard.putNumber("1310/shooter/distanceToHub", distance);
//    log("Speed: " + shooterSubsystem.getShooterVelocity());

    if (operatorInput.getDriverController().getPOV() == 0) {
      testShooterSpeed += 100;
      SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    }
    else if(operatorInput.getDriverController().getPOV() == 180) {
      testShooterSpeed -= 100;
      shooterSubsystem.setShooterVelocity(testShooterSpeed);
      SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    }

    if(operatorInput.getDriverController().getPOV() == 270){
      shooterSubsystem.setShooterVelocity(testShooterSpeed);
    }else shooterSubsystem.setShooterSpeed(0);

//      shooterSubsystem.setKickerSpeed(0.5);
//    } else {
//      shooterSubsystem.setKickerSpeed(0.0);
//    }

    if (operatorInput.getDriverController().getYButtonPressed()) {
      shooterSubsystem.setKickerSpeed(0.7);
    } else shooterSubsystem.setKickerSpeed(0);
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
//      log("Target speed: " + shooterSpeed);
    }
    return shooterSpeed;
  }


  public void shooting(double distance){
    double shooterSpeed = calculateShootingSpeed(distance) - 20;

    log("actual: " + shooterSubsystem.getShooterVelocity());

    shooterSubsystem.setShooterVelocity(shooterSpeed);
    if(shooterSubsystem.getShooterVelocity() >= shooterSpeed){
      shooterSubsystem.setKickerSpeed(0.5);
    } else shooterSubsystem.setKickerSpeed(0);
  }

}
