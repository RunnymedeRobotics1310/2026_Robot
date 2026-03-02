package frc.robot.commands;

import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

public class IntakeCommand extends LoggingCommand{

    private final IntakeSubsystem intakeSubsystem;
    private final OperatorInput oi;


    public IntakeCommand(IntakeSubsystem intake, OperatorInput operatorInput){
        super();
        intakeSubsystem = intake;
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
        //door go out
        //spin motors
        //only when you press a button
        final boolean intakeCheck = oi.isIntakeDoingStuff();

        if(intakeCheck) {
            intakeSubsystem.setRollers(true);
            intakeSubsystem.setDoorState(true);

        } else {
            intakeSubsystem.rollerStop();
            intakeSubsystem.setDoorState(false);
        }

        if (oi.isReverseIntake()) {
          intakeSubsystem.setRollerSpeeds(-INTAKE_SPEED, -INTAKE_SPEED);
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

