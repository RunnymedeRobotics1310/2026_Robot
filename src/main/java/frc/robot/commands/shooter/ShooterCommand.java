package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.SLOPE_VALUE;
import static frc.robot.Constants.ShooterConstants.Y_INT;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final LimelightVisionSubsystem vision;

  private final OperatorInput operatorInput;

  private final Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem vision,
                        OperatorInput operatorInput, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.vision = vision;
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
    double distance = swerveSubsystem.distanceToHub();
    SmartDashboard.putNumber("1310/shooter/distanceToHub", distance);
    log("Speed: " + shooterSubsystem.getShooterVelocity());

    shooting(distance);
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
    timer.stop();
    timer.reset();
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    if (distanceMeters < 10.0) {
      shooterSpeed = (distanceMeters * SLOPE_VALUE) + Y_INT;
      // log("Target speed: " + shooterSpeed);
    }
    return shooterSpeed;
  }

  public void shooting(double distance) {
    double shooterSpeed = calculateShootingSpeed(distance);
    shooterSubsystem.setShooterVelocity(shooterSpeed);
    SmartDashboard.putNumber("1310/shooter/targetspeed", shooterSpeed);

    // Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed) < 10

    if (timer.hasElapsed(2.5)) {
      shooterSubsystem.setKickerSpeed(-0.7);
    }
    if (timer.hasElapsed(2.8)) {
      shooterSubsystem.setKickerSpeed(0.0);
      timer.reset();
      timer.stop();
    }
  }
}
