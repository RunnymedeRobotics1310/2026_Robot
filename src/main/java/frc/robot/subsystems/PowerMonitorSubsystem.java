package frc.robot.subsystems;

import static frc.robot.Constants.PowerConstants.*;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerConstants.PDHChannel;
import frc.robot.telemetry.Telemetry;

public class PowerMonitorSubsystem extends SubsystemBase {

  private final PowerDistribution pdh = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

  // DataLog entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry totalCurrentLog = new DoubleLogEntry(log, "Power/TotalCurrent");
  private final DoubleLogEntry voltageLog = new DoubleLogEntry(log, "Power/Voltage");
  private final DoubleLogEntry temperatureLog = new DoubleLogEntry(log, "Power/Temperature");
  private final DoubleLogEntry[] channelLogs = new DoubleLogEntry[24];

  // Per-channel sustained overcurrent timers (time when threshold was first exceeded)
  private final double[] warningExceededSince = new double[24];
  private final double[] errorExceededSince = new double[24];

  // Alerts for monitored channels
  private final Alert[] channelWarningAlerts;
  private final Alert[] channelErrorAlerts;

  // Main breaker alerts
  private final Alert mainBreakerWarning =
      new Alert("Total current approaching main breaker limit", AlertType.kWarning);
  private final Alert mainBreakerError =
      new Alert("Total current at main breaker limit!", AlertType.kError);

  // Voltage alerts
  private final Alert voltageLowWarning = new Alert("Battery voltage low", AlertType.kWarning);
  private final Alert voltageCriticalError =
      new Alert("Battery voltage critical!", AlertType.kError);

  public PowerMonitorSubsystem() {
    // Initialize data log entries for all 24 channels
    for (int i = 0; i < 24; i++) {
      channelLogs[i] = new DoubleLogEntry(log, "Power/Channel" + i);
      warningExceededSince[i] = 0;
      errorExceededSince[i] = 0;
    }

    // Create alerts for monitored channels
    channelWarningAlerts = new Alert[MONITORED_CHANNELS.length];
    channelErrorAlerts = new Alert[MONITORED_CHANNELS.length];
    for (int i = 0; i < MONITORED_CHANNELS.length; i++) {
      PDHChannel ch = MONITORED_CHANNELS[i];
      channelWarningAlerts[i] =
          new Alert(
              ch.deviceName() + " (ch" + ch.port() + ") approaching breaker limit",
              AlertType.kWarning);
      channelErrorAlerts[i] =
          new Alert(ch.deviceName() + " (ch" + ch.port() + ") at breaker limit!", AlertType.kError);
    }
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double totalCurrent = pdh.getTotalCurrent();
    double voltage = pdh.getVoltage();
    double temperature = pdh.getTemperature();

    // Read all channel currents
    double[] currents = new double[24];
    for (int i = 0; i < 24; i++) {
      currents[i] = pdh.getCurrent(i);
    }

    // --- Peak tracking ---
    if (totalCurrent > Telemetry.power.peakTotalCurrentAmps) {
      Telemetry.power.peakTotalCurrentAmps = totalCurrent;
    }
    for (int i = 0; i < 24; i++) {
      if (currents[i] > Telemetry.power.peakChannelCurrents[i]) {
        Telemetry.power.peakChannelCurrents[i] = currents[i];
      }
    }

    // --- DataLog ---
    totalCurrentLog.append(totalCurrent);
    voltageLog.append(voltage);
    temperatureLog.append(temperature);
    for (int i = 0; i < 24; i++) {
      channelLogs[i].append(currents[i]);
    }

    // --- Per-channel sustained overcurrent detection ---
    int warningCount = 0;
    int errorCount = 0;

    for (int i = 0; i < MONITORED_CHANNELS.length; i++) {
      PDHChannel ch = MONITORED_CHANNELS[i];
      int port = ch.port();
      double current = currents[port];
      double warnThreshold = ch.breaker().warningAmps();
      double errorThreshold = ch.breaker().errorAmps();

      // Error tier (100% of rated)
      if (current >= errorThreshold) {
        if (errorExceededSince[port] == 0) {
          errorExceededSince[port] = now;
        }
        if (now - errorExceededSince[port] >= SUSTAINED_OVERCURRENT_SECONDS) {
          channelErrorAlerts[i].set(true);
          errorCount++;
        }
      } else {
        errorExceededSince[port] = 0;
        channelErrorAlerts[i].set(false);
      }

      // Warning tier (85% of rated)
      if (current >= warnThreshold) {
        if (warningExceededSince[port] == 0) {
          warningExceededSince[port] = now;
        }
        if (now - warningExceededSince[port] >= SUSTAINED_OVERCURRENT_SECONDS) {
          channelWarningAlerts[i].set(true);
          warningCount++;
        }
      } else {
        warningExceededSince[port] = 0;
        channelWarningAlerts[i].set(false);
      }
    }

    // --- Main breaker monitoring ---
    mainBreakerError.set(totalCurrent >= MAIN_BREAKER_ERROR_AMPS);
    mainBreakerWarning.set(
        totalCurrent >= MAIN_BREAKER_WARNING_AMPS && totalCurrent < MAIN_BREAKER_ERROR_AMPS);
    if (totalCurrent >= MAIN_BREAKER_ERROR_AMPS) {
      errorCount++;
    } else if (totalCurrent >= MAIN_BREAKER_WARNING_AMPS) {
      warningCount++;
    }

    // --- Voltage monitoring ---
    voltageCriticalError.set(voltage <= VOLTAGE_CRITICAL_VOLTS);
    voltageLowWarning.set(voltage <= VOLTAGE_WARNING_VOLTS && voltage > VOLTAGE_CRITICAL_VOLTS);
    if (voltage <= VOLTAGE_CRITICAL_VOLTS) {
      errorCount++;
    } else if (voltage <= VOLTAGE_WARNING_VOLTS) {
      warningCount++;
    }

    // --- Update telemetry ---
    Telemetry.power.totalCurrentAmps = totalCurrent;
    Telemetry.power.voltageVolts = voltage;
    Telemetry.power.temperatureCelsius = temperature;
    Telemetry.power.channelCurrents = currents;
    Telemetry.power.activeWarningCount = warningCount;
    Telemetry.power.activeErrorCount = errorCount;
  }
}
