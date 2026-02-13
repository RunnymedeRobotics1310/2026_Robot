package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.ShooterConstants.*;

/** An example command that uses an example subsystem. */
public class TuneShooterCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final OperatorInput operatorInput;

  private double testShooterSpeed;

  private int lastPov = -1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public TuneShooterCommand(ShooterSubsystem shooterSubsystem,
                            OperatorInput operatorInput,
                            SwerveSubsystem swerveSubsystem) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean YButton = operatorInput.getDriverController().getYButton();
    int currentPOV = operatorInput.getDriverController().getPOV();
    double distance = swerveSubsystem.distanceToHub();
    SmartDashboard.putNumber("1310/shooter/distanceToHub", distance);

     if (currentPOV == 0 && lastPov == -1) {
       testShooterSpeed = Math.min(testShooterSpeed + 20, MAX_SHOOTER_RPM);
       SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
     }

     if (currentPOV == 180 && lastPov == -1) {
       testShooterSpeed = Math.max(testShooterSpeed - 20, 0);
       SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
     }

     if (YButton) {
       shooterSubsystem.setShooterVelocity(testShooterSpeed);
//       shooterSubsystem.setShooterSpeed(testShooterSpeed);
       SmartDashboard.putNumber("1310/shooter/currentspeed",
       shooterSubsystem.getShooterVelocity());
     } else {
       shooterSubsystem.setShooterSpeed(0.0);
     }

     if (currentPOV == 270) {
       shooterSubsystem.setKickerSpeed(-0.7);
     } else {
       shooterSubsystem.setKickerSpeed(0.0);
     }

    // hood control
    if (currentPOV == 90) {
      shooterSubsystem.setHood(operatorInput.getDriverController().getLeftTriggerAxis());
    }

    lastPov = currentPOV;

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
    shooterSubsystem.stop();
  }
}