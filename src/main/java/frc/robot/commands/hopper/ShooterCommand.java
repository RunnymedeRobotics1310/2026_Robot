package frc.robot.commands.hopper;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.Constants.ShooterConstants.AGITATOR_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.CLOSE_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;
import static frc.robot.Constants.ShooterConstants.MAX_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.MEDIUM_SHOOT_HOOD_VALUE;
import static frc.robot.Constants.ShooterConstants.SUPER_FAR_SHOOTING_DISTANCE;
import static frc.robot.Constants.ShooterConstants.SUPER_FAR_SHOOT_HOOD_VALUE;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(
    value = "shooter",
    category = "shooter",
    description = "Auto-aim shooter using distance to hub")
public class ShooterCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final Timer timer = new Timer();
  private final Timer kickerDebounceTimer = new Timer();

  public ShooterCommand(HopperSubsystem hopperSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.reset();
    timer.start();
    kickerDebounceTimer.reset();
    kickerDebounceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooting();
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
    hopperSubsystem.setHood(0);
    hopperSubsystem.stop();
    timer.stop();
    timer.reset();
    kickerDebounceTimer.stop();
    kickerDebounceTimer.reset();
  }

  // public static final double ACCEPTED_THRESHOLD = 200.0;

  public void shooting() {
    double distance = swerveSubsystem.distanceToHub();

    // hood
    double hoodAngle = 0;
    if (distance < MAX_SHOOTING_DISTANCE) {
      if (distance >= SUPER_FAR_SHOOTING_DISTANCE) {
        hoodAngle = SUPER_FAR_SHOOT_HOOD_VALUE;
      } else if (distance >= MEDIUM_SHOOTING_DISTANCE) {
        hoodAngle = MEDIUM_SHOOT_HOOD_VALUE;
      }
    } else {
      hoodAngle = CLOSE_SHOOT_HOOD_VALUE;
    }
    hopperSubsystem.setHood(hoodAngle);

    // shooter
    double targetSpeed = hopperSubsystem.calculateShootingSpeed(distance);
    hopperSubsystem.setShooterVelocity(targetSpeed);

    // agitator
    hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    hopperSubsystem.setRollerSpeeds(0, INTAKE_SPEED);

    // kicker
    boolean atSpeed = hopperSubsystem.isShooterAtSpeed();

    // boolean overThreshold = Math.abs(targetSpeed - currentVelocity) < ACCEPTED_THRESHOLD;
    boolean facingHub =
        SwerveUtils.isCloseEnough(
            swerveSubsystem.angleToShootTowards().getDegrees(), swerveSubsystem.getYaw(), 5);

    if (atSpeed && facingHub) {
      kickerDebounceTimer.reset();
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    } else if (kickerDebounceTimer.hasElapsed(0.1)) { // FIXME change to constant later
      hopperSubsystem.setKickerSpeed(0);
    }
  }
}
