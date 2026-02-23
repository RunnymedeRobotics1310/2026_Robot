package frc.robot.telemetry;

import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RunnymedeUtils;

/**
 * @author Tony Field
 * @since 2025-02-16 10:45
 */
public class ShooterTelemetry {

  public boolean enabled = true;

  /** The top intake roller speed */
  public double topRollerSpeed = Double.MIN_VALUE;

  /** the bottom intake roller speed */
  public double bottomRollerSpeed = Double.MIN_VALUE;

  /** Whether the arm is extended or not */
  public boolean isArmExtended = false;

  /** Whether the hopper is full or not */
  public boolean isHopperFull = false;

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(
          PREFIX + "Intake/TopRollerSpeed", RunnymedeUtils.round(topRollerSpeed));
      SmartDashboard.putNumber(
          PREFIX + "Intake/BottomRollerSpeed", RunnymedeUtils.round(bottomRollerSpeed));
      SmartDashboard.putBoolean(PREFIX + "Intake/ArmExtended", isArmExtended);
      SmartDashboard.putBoolean(PREFIX + "Intake/HopperFull", isHopperFull);
    }
  }
}
