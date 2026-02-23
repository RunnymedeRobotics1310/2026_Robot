package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;
import frc.robot.telemetry.ShooterTelemetry;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends LoggingCommand {

  private final ShooterSubsystem shooterSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final LimelightVisionSubsystem vision;

  private final OperatorInput operatorInput;

  private final Timer timer = new Timer();

  private final double SLOPE_VALUE_FAR = ShooterConstants.SLOPE_VALUE_FAR;
  private final double Y_INT_FAR = ShooterConstants.Y_INT_FAR;
  private final double SLOPE_VALUE_MID = ShooterConstants.SLOPE_VALUE_MID;
  private final double Y_INT_MID = ShooterConstants.Y_INT_MID;
  private final double SLOPE_VALUE_CLOSE = ShooterConstants.SLOPE_VALUE_CLOSE;
  private final double Y_INT_CLOSE = ShooterConstants.Y_INT_CLOSE;

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
    shooterSubsystem.setHood(0);
    shooterSubsystem.stop();
    timer.stop();
    timer.reset();
  }

  public double calculateShootingSpeed(double distanceMeters) {
    double shooterSpeed = 0;
    if (distanceMeters < 10.0) {
      if (distanceMeters > 2.2) {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_FAR) + Y_INT_FAR;
        // log("Target speed: " + shooterSpeed);
      } else if (distanceMeters > 1.5) {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_MID) + Y_INT_MID;
      } else {
        shooterSpeed = (distanceMeters * SLOPE_VALUE_CLOSE) + Y_INT_CLOSE;
      }
    }
    return shooterSpeed;
  }

  public void shooting(double distance) {
    double shooterSpeed = calculateShootingSpeed(distance);
    shooterSubsystem.setShooterVelocity(shooterSpeed);
    SmartDashboard.putNumber("1310/shooter/targetspeed", shooterSpeed);
    if (swerveSubsystem.distanceToHub() > 2.2) {
      shooterSubsystem.setHood(1.0);
    } else if (swerveSubsystem.distanceToHub() > 1.5) {
      shooterSubsystem.setHood(0.6);
    }
    // else {
    // shooterSubsystem.setHood(0.0);
    // }
    // Math.abs(shooterSubsystem.getShooterVelocity() - shooterSpeed) < 10

    if (timer.hasElapsed(1.75)) {
      shooterSubsystem.setKickerSpeed(-0.7);
      ShooterTelemetry.isShooting = true;
    }
    if (timer.hasElapsed(2.0)) {
      shooterSubsystem.setKickerSpeed(0.0);
      ShooterTelemetry.isShooting = false;
      timer.reset();
      timer.stop();
    }
  }
}
