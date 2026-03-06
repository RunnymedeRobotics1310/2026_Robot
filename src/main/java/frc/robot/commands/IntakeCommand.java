package frc.robot.commands;

import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.IntakeConstants.INTAKE_DOOR_ANGLE;

public class IntakeCommand extends LoggingCommand {

  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooter;
  private final OperatorInput oi;

  public IntakeCommand(
      IntakeSubsystem intake, ShooterSubsystem shooter, OperatorInput operatorInput) {
    super();
    intakeSubsystem = intake;
    this.shooter = shooter;
    addRequirements(intake);
    oi = operatorInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // door go out
    // spin motors
    // only when you press a button
    final boolean intakeCheck = oi.isIntakeDoingStuff();

    //        if(intakeCheck) {
    //            intakeSubsystem.setRollers(true);
    //            intakeSubsystem.setDoorState(true);
    //
    //        } else {
    //            intakeSubsystem.rollerStop();
    //            intakeSubsystem.setDoorState(false);
    //        }
    //
    //        if (oi.isReverseIntake()) {
    //          intakeSubsystem.setRollerSpeeds(-INTAKE_SPEED, -INTAKE_SPEED);
    //        }

    if (intakeCheck) {
      intakeSubsystem.setRollerSpeeds(-1, -0.8);
      intakeSubsystem.setDoorSetpoint(INTAKE_DOOR_ANGLE);
      //      shooter.setAgitatorSpeed(Constants.ShooterConstants.AGITATOR_RUNSPEED);
    } else {
      intakeSubsystem.setRollerSpeeds(0, 0);
      intakeSubsystem.setDoorSetpoint(0);
      shooter.stop();
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
    intakeSubsystem.rollerStop();
  }
}
