package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ConfigIntakeCommand extends LoggingCommand {

    private final IntakeSubsystem intake;
    private final AutoStep.IntakeAction action;
    private final double speed;
    private final double durationSeconds;

    private final Timer timer = new Timer();

    public ConfigIntakeCommand(
            IntakeSubsystem intake,
            AutoStep.IntakeAction action,
            double speed,
            double durationSeconds) {
        this.intake = intake;
        this.action = action;
        this.speed = speed;
        this.durationSeconds = durationSeconds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        logCommandStart("action=" + action + " speed=" + speed);
        timer.reset();
        timer.start();

        if (action == null || action == AutoStep.IntakeAction.off) {
            intake.stop();
        } else {
            intake.setSpeed(speed);
        }
    }

    @Override
    public void execute() {
        if (action != null && action != AutoStep.IntakeAction.off) {
            intake.setSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        if (action == null) {
            setFinishReason("Invalid intake action");
            return true;
        }
        if (action == AutoStep.IntakeAction.off) {
            setFinishReason("Intake off");
            return true;
        }
        if (action == AutoStep.IntakeAction.on_with_duration && timer.hasElapsed(durationSeconds)) {
            setFinishReason("Duration elapsed: " + durationSeconds + "s");
            return true;
        }
        // "on" action runs until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (action != AutoStep.IntakeAction.on) {
            intake.stop();
        }
        logCommandEnd(interrupted);
    }
}
