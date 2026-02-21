package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class CloseShootCommand extends LoggingCommand {

    private final ShooterSubsystem shooterSubsystem;

    private final SwerveSubsystem swerveSubsystem;

    private final OperatorInput operatorInput;

    private final Timer timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     *
     * @param shooterSubsystem The subsystem used by this command.
     */
    public CloseShootCommand(ShooterSubsystem shooterSubsystem,
            OperatorInput operatorInput, SwerveSubsystem swerveSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.operatorInput = operatorInput;
        this.swerveSubsystem = swerveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
        timer.start();
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shootClose();
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
        shooterSubsystem.setHood(0);
        shooterSubsystem.stop();
        timer.stop();
        timer.reset();
    }

    public void shootClose() {
        shooterSubsystem.setHood(0.0);
        int targetspeed = 2900;
        shooterSubsystem.setShooterVelocity(targetspeed);
        if (shooterSubsystem.getShooterVelocity() > 2850) {
            shooterSubsystem.setKickerSpeed(0.7);
        } else {
            shooterSubsystem.setKickerSpeed(0.0);
        }

    }

}
