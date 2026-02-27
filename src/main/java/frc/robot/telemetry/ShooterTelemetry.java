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

  public double targetShooterRPM;
  public double currentShooterRPM;
  public double kickerSpeed;
  public double hoodAngle;

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(PREFIX + "Shooter/TargetShooterRPM", (int) targetShooterRPM);
      SmartDashboard.putNumber(PREFIX + "Shooter/CurrentShooterRPM", (int) currentShooterRPM);
      SmartDashboard.putNumber(PREFIX + "Shooter/KickerSpeed", kickerSpeed);
      SmartDashboard.putNumber(PREFIX + "Shooter/HoodAngle", hoodAngle);
    }
  }
}
