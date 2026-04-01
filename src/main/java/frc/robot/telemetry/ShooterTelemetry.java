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
  public double currentLeftShooterRPM;
  public double currentRightShooterRPM;
  public double kickerSpeed;
  public double hoodAngle;
  public double agitatorSpeed;

  public double leftShooterTemp;
  public double rightShooterTemp;

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(PREFIX + "Shooter/TargetShooterRPM", (int) targetShooterRPM);
      SmartDashboard.putNumber(
          PREFIX + "Shooter/CurrentLeftShooterRPM", (int) currentLeftShooterRPM);
      SmartDashboard.putNumber(
          PREFIX + "Shooter/CurrentRightShooterRPM", (int) currentRightShooterRPM);
      SmartDashboard.putNumber(PREFIX + "Shooter/KickerSpeed", RunnymedeUtils.round(kickerSpeed));
      SmartDashboard.putNumber(PREFIX + "Shooter/HoodAngle", RunnymedeUtils.round(hoodAngle));

      SmartDashboard.putNumber(PREFIX + "Shooter/rightShooterTemp", rightShooterTemp);
      SmartDashboard.putNumber(PREFIX + "Shooter/leftShooterTemp", leftShooterTemp);
    }
  }
}
