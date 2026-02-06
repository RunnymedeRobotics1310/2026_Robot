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
    shooterSubsystem.shooterMotor.set(0.7); // Use for testing
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = swerveSubsystem.distanceToHub();

    log("dist: " + distance);


    if (operatorInput.getDriverController().getPOV() == 0) {
      shooterSubsystem.kickerMotor.set(0.5);
    } else {
      shooterSubsystem.kickerMotor.set(0.0);
    }

    if (operatorInput.getDriverController().getYButtonPressed() == true) {
      shooting(distance);
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
  }

  public double calculateShootingSpeed(double distanceMeters) {
    if (distanceMeters < 10.0) {
      return (distanceMeters * Constants.ShooterConstants.SLOPE_VALUE) + Constants.ShooterConstants.Y_INT; // FIXME add real equation
    } else {
      return 1.0;
    }
  }

  public void setMotorVelocity(double setPoint, SparkFlex motor) {
    double maxShooterRpm = Constants.ShooterConstants.MAX_SHOOTER_RPM;
    double Kp = Constants.ShooterConstants.KP;

    double currentSpeed = motor.getEncoder().getVelocity();
    double error = (setPoint - currentSpeed) / maxShooterRpm; // Normalize error
    motor.set((setPoint / maxShooterRpm) + (error * Kp));
  }

  public void shooting(double distance){
    double shooterSpeed = calculateShootingSpeed(distance);
    SparkFlex shooterMotor = shooterSubsystem.shooterMotor;
    SparkFlex kickerMotor = shooterSubsystem.kickerMotor;

    setMotorVelocity(shooterSpeed, shooterMotor);
    if(shooterMotor.getEncoder().getVelocity() >= shooterSpeed){
      kickerMotor.set(0.7);
    } else kickerMotor.stopMotor();
  }

}
