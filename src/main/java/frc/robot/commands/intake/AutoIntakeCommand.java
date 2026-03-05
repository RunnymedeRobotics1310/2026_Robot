package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.IntakeSubsystem;

@AutoConfigurable(value = "intake", category = "intake",
    description = "Control the intake motor")
public class AutoIntakeCommand extends LoggingCommand {

    private final IntakeSubsystem intake;
    private final String action;
    private final double speed;
    private final double durationSeconds;

    private final Timer timer = new Timer();

    public AutoIntakeCommand(
            IntakeSubsystem intake,
            @ConfigParam(value = "intakeAction", options = {"on", "on_with_duration", "off"},
                description = "Intake action") String action,
            @ConfigParam(value = "speed", min = -1, max = 1,
                description = "Intake motor speed") double speed,
            @ConfigParam(value = "durationSeconds", unit = "s", min = 0, max = 15,
                description = "Duration for on_with_duration action") double durationSeconds) {
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

        if (action == null || "off".equals(action)) {
            intake.stop();
        } else {
            intake.setSpeed(speed);
        }
    }

    @Override
    public void execute() {
        if (action != null && !"off".equals(action)) {
            intake.setSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        if (action == null) {
            setFinishReason("Invalid intake action");
            return true;
        }
        if ("off".equals(action)) {
            setFinishReason("Intake off");
            return true;
        }
        if ("on_with_duration".equals(action) && timer.hasElapsed(durationSeconds)) {
            setFinishReason("Duration elapsed: " + durationSeconds + "s");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (!"on".equals(action)) {
            intake.stop();
        }
        logCommandEnd(interrupted);
    }
}
