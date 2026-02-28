package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.MAX_SHOOTER_RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
  public TuneShooterCommand(
      ShooterSubsystem shooterSubsystem,
      OperatorInput operatorInput,
      SwerveSubsystem swerveSubsystem,
      IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, swerveSubsystem, intakeSubsystem);
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

    if (currentPOV == 0 && lastPov == -1) {
      testShooterSpeed = Math.min(testShooterSpeed + 50, MAX_SHOOTER_RPM);
      SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    }

    if (currentPOV == 180 && lastPov == -1) {
      testShooterSpeed = Math.max(testShooterSpeed - 50, 0);
      SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    }

    if (YButton) {
      shooterSubsystem.setShooterVelocity(testShooterSpeed);

    } else if (operatorInput.getDriverController().getBButton()) {
      shooterSubsystem.setShooterSpeed(1);
    } else {
      shooterSubsystem.setShooterSpeed(0.0);
    }

    if (currentPOV == 270) {
      shooterSubsystem.setKickerSpeed(1);
    } else {
      shooterSubsystem.setKickerSpeed(0.0);
    }

    // hood control
    //    if (currentPOV == 90) {
    //      shooterSubsystem.setHood(operatorInput.getDriverController().getLeftTriggerAxis());
    //    }
    double joystick = operatorInput.getDriverControllerAxis(OperatorInput.Stick.RIGHT, OperatorInput.Axis.Y);

    shooterSubsystem.setHood(Math.abs(joystick));

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
