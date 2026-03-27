package frc.robot.commands.auto;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;

@AutoConfigurable(
    value = "auto_climb",
    category = "climb",
    description = "Extend or retract the climb mechanism")
public class AutoClimbCommand extends LoggingCommand {

  HopperSubsystem hopper;
  ClimbSubsystem climb;

  private final boolean climbGoingUp;

  public AutoClimbCommand(
      ClimbSubsystem climb,
      HopperSubsystem hopper,
      @ConfigParam(
              value = "climbGoingUp",
              description = "True to extend, false to retract",
              defaultValue = 1)
          boolean climbGoingUp) {
    addRequirements(climb);
    addRequirements(hopper);

    this.hopper = hopper;
    this.climb = climb;
    this.climbGoingUp = climbGoingUp;
  }

  @Override
  public void initialize() {
    logCommandStart();
    hopper.setDoorSetpoint(0);
    if (!climbGoingUp && DriverStation.isAutonomousEnabled()) climb.isAutoClimbed = true;
  }

  @Override
  public void execute() {
    hopper.setDoorSetpoint(0);
    if (climbGoingUp) {
      climb.setClimbSpeed(1);
      if (climb.getClimbPosition() > MAX_CLIMB_POSITION - CLIMB_SLOW_ZONE) {
        climb.setClimbSpeed(CLIMB_SLOW_ZONE_SPEED);
      }
    } else {
      climb.setClimbSpeed(-1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    climb.stop();
  }

  @Override
  public boolean isFinished() {
    //    log("CLIMB: " + climb.getPos());

    if (climbGoingUp && climb.getClimbPosition() >= MAX_CLIMB_POSITION) {
      return true;
    }
    if (!climbGoingUp && climb.getClimbPosition() <= 40) {
      return true;
    }
    return false;
  }
}
