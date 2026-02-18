package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class LazyShooterCommand extends LoggingCommand {

    private final ShooterSubsystem shooterSubsystem;

    private final int speed;

    private final Timer timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     *
     * @param shooterSubsystem The subsystem used by this command.
     */
    public LazyShooterCommand(ShooterSubsystem shooterSubsystem,
            int speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        lazyShoot(speed);
        log("Speed: " + shooterSubsystem.getShooterVelocity());

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timer.get() > 4.0) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        shooterSubsystem.stop();
        timer.stop();
        timer.reset();
    }

    public void lazyShoot(double shootRpm) {
        shooterSubsystem.setShooterVelocity(shootRpm);
        if (timer.hasElapsed(2.0)) {
            shooterSubsystem.setKickerSpeed(-0.7);
        } else {
            shooterSubsystem.setKickerSpeed(0);
        }

    }
}
