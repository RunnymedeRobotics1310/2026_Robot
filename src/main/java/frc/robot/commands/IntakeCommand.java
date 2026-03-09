package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.INTAKE_DOOR_ANGLE;
import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends LoggingCommand {

  private final IntakeSubsystem intakeSubsystem;
  private final OperatorInput oi;
  boolean firstIntake;

  private Timer timer = new Timer();

  public IntakeCommand(IntakeSubsystem intake, OperatorInput operatorInput) {
    super();
    intakeSubsystem = intake;
    addRequirements(intake);
    oi = operatorInput;
    firstIntake = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // door go out
    // spin motors
    // only when you press a button

    final boolean intakeCheck = oi.isIntakeDoingStuff();

    if (intakeCheck) {
      intakeSubsystem.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
      intakeSubsystem.setDoorSetpoint(INTAKE_DOOR_ANGLE);
      firstIntake = true;
      timer.reset();
    } else {
      intakeSubsystem.setDoorSetpoint(0);
      if (timer.get() < 0.5 && firstIntake) {
        intakeSubsystem.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
      } else {
        intakeSubsystem.setRollerSpeeds(0, 0);
      }
    }

    // operator overrides
    if (oi.isIntakeForwards()) intakeSubsystem.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
    if (oi.isIntakeReverse()) intakeSubsystem.setRollerSpeeds(-INTAKE_SPEED, -INTAKE_SPEED);
    if (oi.isOpenDoor()) intakeSubsystem.setDoorSetpoint(90);
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
    firstIntake = false;
    timer.reset();
    timer.stop();
  }
}
