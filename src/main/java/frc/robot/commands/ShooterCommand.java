// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final LightingSubsystem lightingSubsystem;

  private final ShooterSubsystem shooterSubsystem;

  private final LimelightVisionSubsystem vision;

  private final OperatorInput operatorInput;

  private Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem vision, LightingSubsystem lightingSubsystem,
      OperatorInput operatorInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.lightingSubsystem = lightingSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.vision = vision;
    this.operatorInput = operatorInput;
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

    double distance = -1310;

    if (vision.isTagInView(26)) {
      log("26 in view");
      distance = vision.distanceTagToRobot(26);
    } else if (vision.isTagInView(24)) {
      distance = vision.distanceTagToRobot(24);
    } else if (vision.isTagInView(18)) {
      distance = vision.distanceTagToRobot(18);
    } else {
      distance = vision.distanceTagToRobot(0);
    }
//    log(vision.angleToTarget(0));
//    distance = vision.distanceTagToRobot(tag);
//
//    log("visible: " + tag);
//
    log("dist: " + distance);

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
}
