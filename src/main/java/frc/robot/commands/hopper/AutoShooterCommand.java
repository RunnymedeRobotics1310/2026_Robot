package frc.robot.commands.hopper;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.Constants.ShooterConstants.KICKER_RUNSPEED;

import ca.team1310.swerve.utils.SwerveUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(
    value = "shooter",
    category = "shooter",
    description = "Auto-aim shooter using distance to hub")
public class AutoShooterCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;

  private final SwerveSubsystem swerveSubsystem;

  private final int shots;

  private boolean firstShot = false;

  public AutoShooterCommand(
      HopperSubsystem hopperSubsystem, SwerveSubsystem swerveSubsystem, int shots) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.shots = shots;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    firstShot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopperSubsystem.getAutoShotCount() >= shots;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    hopperSubsystem.setHood(0);
    hopperSubsystem.stop();
  }

  // public static final double ACCEPTED_THRESHOLD = 200.0;

  public void shooting() {
    // shooter
    double distance = swerveSubsystem.distanceToHub();
    double targetSpeed = hopperSubsystem.calculateShootingSpeed(distance);
    hopperSubsystem.setShooterVelocity(targetSpeed);

    // agitator
    //    hopperSubsystem.setAgitatorSpeed(AGITATOR_RUNSPEED);
    //    hopperSubsystem.reverseAgitator(1);
    hopperSubsystem.pulseAgitator(1);
    hopperSubsystem.setRollerSpeeds(0, INTAKE_SPEED);

    // hood
    hopperSubsystem.setHood(hopperSubsystem.calculateHoodValule(distance));

    // kicker
    boolean atSpeed = hopperSubsystem.isShooterAtSpeed();
    boolean facingHub =
        SwerveUtils.isCloseEnough(swerveSubsystem.getHubAngleDeg(), swerveSubsystem.getYaw(), 5);

    if ((atSpeed || firstShot) && facingHub) {
      firstShot = true;
      hopperSubsystem.setKickerSpeed(KICKER_RUNSPEED);
    } else {
      hopperSubsystem.setKickerSpeed(0);
    }
  }
}
