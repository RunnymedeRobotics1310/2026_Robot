package frc.robot.telemetry;

import static frc.robot.RunnymedeUtils.round;
import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Tony Field
 * @since 2025-02-16 10:45
 */
public class ClimbTelemetry {

  public boolean enabled = true;

  /** The climb encoder count */
  public double climbPosition = 0;

  /** The current speed of the climb motor */
  public double climbSpeed = 0;

  /** Whether the climb is at the lower limit */
  public boolean climbDown = false;

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(PREFIX + "Climb/ClimbPosition", round(climbPosition));
      SmartDashboard.putNumber(PREFIX + "Climb/ClimbSpeed", round(climbSpeed));
      SmartDashboard.putBoolean(PREFIX + "Intake/LowerLimit", climbDown);
    }
  }
}
