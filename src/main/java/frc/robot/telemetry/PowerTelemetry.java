package frc.robot.telemetry;

import static frc.robot.RunnymedeUtils.round;
import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerTelemetry {

  public boolean enabled = true;

  public double totalCurrentAmps;
  public double voltageVolts;
  public double temperatureCelsius;
  public double peakTotalCurrentAmps;
  public int activeWarningCount;
  public int activeErrorCount;
  public double[] channelCurrents = new double[24];
  public double[] peakChannelCurrents = new double[24];

  void post() {

    if (enabled) {
      SmartDashboard.putNumber(PREFIX + "Power/TotalCurrent", round(totalCurrentAmps));
      SmartDashboard.putNumber(PREFIX + "Power/Voltage", round(voltageVolts));
      SmartDashboard.putNumber(PREFIX + "Power/Temperature", round(temperatureCelsius));
      SmartDashboard.putNumber(PREFIX + "Power/PeakTotalCurrent", round(peakTotalCurrentAmps));
      SmartDashboard.putNumber(PREFIX + "Power/ActiveWarnings", activeWarningCount);
      SmartDashboard.putNumber(PREFIX + "Power/ActiveErrors", activeErrorCount);

      for (int i = 0; i < channelCurrents.length; i++) {
        SmartDashboard.putNumber(
            PREFIX + "Power/Channel" + i + "/Current", round(channelCurrents[i]));
        SmartDashboard.putNumber(
            PREFIX + "Power/Channel" + i + "/Peak", round(peakChannelCurrents[i]));
      }
    }
  }
}
