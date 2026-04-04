package frc.robot.telemetry;

import static frc.robot.RunnymedeUtils.round;
import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Tony Field
 * @since 2025-02-16 10:45
 */
public class ShooterTelemetry {

  public boolean enabled = true;

  public double targetShooterRPM;
  public double currentLeftShooterRPM;
  public double currentRightShooterRPM;
  public double kickerTarget;
  public double kickerSpeed;
  public double hoodAngle;
  public double agitatorSpeed;
  public boolean facingHub;

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(PREFIX + "Shooter/TargetShooterRPM", (int) targetShooterRPM);
      SmartDashboard.putNumber(
          PREFIX + "Shooter/CurrentLeftShooterRPM", (int) currentLeftShooterRPM);
      SmartDashboard.putNumber(
          PREFIX + "Shooter/CurrentRightShooterRPM", (int) currentRightShooterRPM);
      SmartDashboard.putNumber(PREFIX + "Shooter/KickerTarget", round(kickerTarget));
      SmartDashboard.putNumber(PREFIX + "Shooter/KickerSpeed", round(kickerSpeed));
      SmartDashboard.putNumber(PREFIX + "Shooter/HoodAngle", round(hoodAngle));
      SmartDashboard.putBoolean(PREFIX + "Shooter/facingHub", facingHub);
    }
  }
}
