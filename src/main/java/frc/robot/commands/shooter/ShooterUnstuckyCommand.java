package frc.robot.commands.shooter;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.HopperSubsystem;


public class ShooterUnstuckyCommand extends LoggingCommand {

    private final HopperSubsystem hopperSubsystem;

    public ShooterUnstuckyCommand(HopperSubsystem hopperSubsystem){
        addRequirements(hopperSubsystem);
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        unstuckShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        hopperSubsystem.stop();
    }

    public void unstuckShooter(){
        hopperSubsystem.setKickerSpeed(1);
    }


}
