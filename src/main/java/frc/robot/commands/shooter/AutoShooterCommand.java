package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.ShooterSubsystem;

@AutoConfigurable(value = "shooter", category = "shooter",
    description = "Control the shooter motor, hood, and kicker")
public class AutoShooterCommand extends LoggingCommand {

    private final ShooterSubsystem shooter;
    private final String action;
    private final double rpm;
    private final double hoodPosition;
    private final double kickerSpeed;
    private final double kickerDelaySeconds;
    private final double durationSeconds;

    private final Timer timer = new Timer();
    private boolean kickerStarted = false;

    public AutoShooterCommand(
            ShooterSubsystem shooter,
            @ConfigParam(value = "action", options = {"on", "on_with_duration", "off"},
                description = "Shooter action") String action,
            @ConfigParam(value = "rpm", unit = "rpm", min = 0, max = 6200,
                description = "Shooter RPM") double rpm,
            @ConfigParam(value = "hoodPosition", min = 0, max = 1,
                description = "Hood servo position") double hoodPosition,
            @ConfigParam(value = "kickerSpeed", min = -1, max = 1,
                description = "Kicker motor speed") double kickerSpeed,
            @ConfigParam(value = "kickerDelaySeconds", unit = "s", min = 0, max = 5,
                description = "Delay before starting kicker") double kickerDelaySeconds,
            @ConfigParam(value = "durationSeconds", unit = "s", min = 0, max = 15,
                description = "Duration for on_with_duration action") double durationSeconds) {
        this.shooter = shooter;
        this.action = action;
        this.rpm = rpm;
        this.hoodPosition = hoodPosition;
        this.kickerSpeed = kickerSpeed;
        this.kickerDelaySeconds = kickerDelaySeconds;
        this.durationSeconds = durationSeconds;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        logCommandStart("action=" + action + " rpm=" + rpm + " hood=" + hoodPosition);
        timer.reset();
        timer.start();
        kickerStarted = false;

        if (action == null || "off".equals(action)) {
            shooter.stop();
        } else {
            shooter.setShooterVelocity(rpm);
            shooter.setHood(hoodPosition);
        }
    }

    @Override
    public void execute() {
        if (action == null || "off".equals(action)) {
            return;
        }

        shooter.setShooterVelocity(rpm);

        if (!kickerStarted && kickerDelaySeconds > 0 && timer.hasElapsed(kickerDelaySeconds)) {
            shooter.setKickerSpeed(kickerSpeed);
            kickerStarted = true;
        } else if (kickerDelaySeconds <= 0 && !kickerStarted) {
            shooter.setKickerSpeed(kickerSpeed);
            kickerStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (action == null) {
            setFinishReason("Invalid shooter action");
            return true;
        }
        if ("off".equals(action)) {
            setFinishReason("Shooter off");
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
            shooter.stop();
        }
        logCommandEnd(interrupted);
    }
}
