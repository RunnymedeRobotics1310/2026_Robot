package frc.robot.commands.hopper;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ChaoticShootCommand extends LoggingCommand {

  private final HopperSubsystem hopperSubsystem;
  private final SwerveSubsystem swerve;

  private int cycleCount = 0;
  private int speed = 0;
  private double hoodAngle = 0;
  private double direction = +1;

  private final int MIN_SPEED = 4000;
  private final int MAX_SPEED = 6000;
  private final double MIN_HOOD = 0.7;
  private final double MAX_HOOD = 1;
  private final double YAW_SPREAD = 20;

  private double yawAtStart;

  private final Timer timer = new Timer();

  public ChaoticShootCommand(SwerveSubsystem swerve, HopperSubsystem hopperSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, hopperSubsystem);

    this.swerve = swerve;
    this.hopperSubsystem = hopperSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logCommandStart();
    timer.reset();
    timer.start();

    yawAtStart = swerve.getYaw();
    swerve.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hopperSubsystem.pulseAgitator(1);
    hopperSubsystem.setRollerSpeeds(0, INTAKE_SPEED);

    if (timer.hasElapsed(0.5)) {
      hopperSubsystem.setKickerSpeed(.5);
    }

    if (cycleCount++ > 15) {
      cycleCount = 0;
      speed = (int) (MIN_SPEED + Math.random() * (MAX_SPEED - MIN_SPEED));
      hoodAngle = MIN_HOOD + Math.random() * (MAX_HOOD - MIN_HOOD);
    }

    hopperSubsystem.setHood(hoodAngle);
    hopperSubsystem.setShooterVelocity(speed);

    // === SWERVE ===
    if (swerve.getYaw() > YAW_SPREAD) direction = -1;
    if (swerve.getYaw() < -YAW_SPREAD) direction = +1;

    swerve.driveFieldOriented(0, 0, direction * 0.7);
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
    hopperSubsystem.setHood(0);
    timer.stop();
    timer.reset();
    swerve.stop();
    swerve.setYaw(yawAtStart + swerve.getYaw());
  }
}
