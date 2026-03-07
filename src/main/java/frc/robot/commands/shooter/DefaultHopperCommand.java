package frc.robot.commands.shooter;

import static frc.robot.Constants.IntakeConstants.INTAKE_DOOR_ANGLE;
import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;

import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.HopperSubsystem;

public class DefaultHopperCommand extends LoggingCommand {

    private final HopperSubsystem hopper;
    private final OperatorInput oi;

    private int rollerDelayCount = 0;

    public DefaultHopperCommand(HopperSubsystem hopper, OperatorInput oi) {
        this.hopper = hopper;
        this.oi = oi;
        addRequirements(hopper);
    }

    @Override
    public void execute() {

        if (oi.isStopFlywheel()) hopper.setShooterSpeed(-0.01);
        if (oi.isReverseKicker()) hopper.setKickerSpeed(-KICKER_RUNSPEED);
        if (oi.aggravateJackson()) hopper.setAgitatorSpeed(AGITATOR_RUNSPEED);

        if (oi.isIntakeDoingStuff()) {
            hopper.setRollerSpeeds(-1, -0.8);
            hopper.setDoorSetpoint(INTAKE_DOOR_ANGLE);
            rollerDelayCount = 25;
        } else if (oi.isReverseIntake()) {
            hopper.setRollerSpeeds(-INTAKE_SPEED, -INTAKE_SPEED);
            hopper.setDoorSetpoint(90);
        } else {
            hopper.setDoorSetpoint(0);
            if (rollerDelayCount > 0) {
                hopper.setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
            } else {
                hopper.setRollerSpeeds(0, 0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stop();
    }
}
