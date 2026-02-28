package frc.robot.commands.shooter;

import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;

public class DefaultShooterCommand extends LoggingCommand {

    private final ShooterSubsystem shooter;
    private final OperatorInput oi;

    public DefaultShooterCommand(ShooterSubsystem shooter, OperatorInput oi) {
        this.shooter = shooter;
        this.oi = oi;
    }

    @Override
    public void execute() {

        if (oi.isStopFlywheel()) shooter.setShooterSpeed(-0.01);
        if (oi.isReverseKicker()) shooter.setKickerSpeed(-KICKER_RUNSPEED);

    }

}
