package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;

import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends LoggingCommand {

    private final ShooterSubsystem shooter;
    private final OperatorInput oi;

    public DefaultShooterCommand(ShooterSubsystem shooter, OperatorInput oi) {
        this.shooter = shooter;
        this.oi = oi;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

        if (oi.isStopFlywheel()) shooter.setShooterSpeed(-0.01);
        if (oi.isReverseKicker()) shooter.setKickerSpeed(-KICKER_RUNSPEED);
        if (oi.aggravateJackson()) shooter.setAgitatorSpeed(AGITATOR_RUNSPEED);

    }

}
