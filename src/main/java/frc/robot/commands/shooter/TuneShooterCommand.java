package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTER_RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class TuneShooterCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final OperatorInput operatorInput;

  private double testShooterSpeed;

  private int lastPov = -1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param hopperSubsystem The subsystem used by this command.
   */
  public TuneShooterCommand(
      HopperSubsystem hopperSubsystem,
      OperatorInput operatorInput,
      SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem, swerveSubsystem);
    this.hopperSubsystem = hopperSubsystem;
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
    boolean shoot = operatorInput.getDriverController().getLeftTriggerAxis() > 0.5;
    int currentPOV = operatorInput.getDriverController().getPOV();

    if (currentPOV == 0 && lastPov == -1) {
      testShooterSpeed = Math.min(testShooterSpeed + 50, MAX_SHOOTER_RPM);
      SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    }

    if (currentPOV == 180 && lastPov == -1) {
      testShooterSpeed = Math.max(testShooterSpeed - 50, 0);
      SmartDashboard.putNumber("1310/shooter/testrpm", testShooterSpeed);
    }

    if (shoot) {
      hopperSubsystem.setShooterVelocity(testShooterSpeed);

    } else if (operatorInput.getDriverController().getBButton()) {
      hopperSubsystem.setShooterSpeed(1);
    } else {
      hopperSubsystem.setShooterSpeed(0.0);
    }

    if (currentPOV == 270) {
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
      hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    } else {
      hopperSubsystem.setKickerSpeed(0.0);
      hopperSubsystem.setAgitatorSpeed(0.0);
    }

    if (currentPOV == 90) {
      double joystick = operatorInput.getDriverControllerAxis(OperatorInput.Stick.RIGHT, OperatorInput.Axis.Y);
      hopperSubsystem.setHood(Math.abs(joystick));
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
    hopperSubsystem.stop();
  }
}
