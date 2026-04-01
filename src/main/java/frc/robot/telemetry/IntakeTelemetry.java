package frc.robot.telemetry;

import static frc.robot.RunnymedeUtils.round;
import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Tony Field
 * @since 2025-02-16 10:45
 */
public class IntakeTelemetry {

  public boolean enabled = true;

  /** The top intake roller speed */
  public double topRollerSpeed = 0;

  /** the bottom intake roller speed */
  public double bottomRollerSpeed = 0;

  /** Whether the door is at the lower limit or not */
  public boolean isDoorClosed = false;

  /** The current angle of the door in degrees */
  public double doorSetpoint = 0;

  /** The current angle of the door in degrees */
  public double doorAngle = 0;

  /** Whether the hopper is full or not */
  public boolean isHopperFull = false;

  public double topRollerTemp = 0;
  public double bottomRollerTemp = 0;

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(PREFIX + "Intake/TopRollerSpeed", round(topRollerSpeed));
      SmartDashboard.putNumber(PREFIX + "Intake/BottomRollerSpeed", round(bottomRollerSpeed));
      SmartDashboard.putBoolean(PREFIX + "Intake/DoorLowerLimit", isDoorClosed);
      SmartDashboard.putNumber(PREFIX + "Intake/DoorSetpoint", doorSetpoint);
      SmartDashboard.putNumber(PREFIX + "Intake/DoorAngle", round(doorAngle));
      SmartDashboard.putBoolean(PREFIX + "Intake/HopperFull", isHopperFull);
      SmartDashboard.putNumber(PREFIX + "Intake/TopRollerTemp", topRollerTemp);
      SmartDashboard.putNumber(PREFIX + "Intake/BottomRollerTemp", bottomRollerTemp);
    }
  }
}
