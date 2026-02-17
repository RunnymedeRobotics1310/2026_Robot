package frc.robot.telemetry;

import ca.team1310.swerve.SwerveTelemetry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

  public enum AlertLevel {
    NONE,
    WARNING,
    ERROR
  }

  public static final String PREFIX = "1310/";

  public static Test test = new Test();
  public static SwerveTelemetry swerve = null;
  public static DriveTelemetry drive = new DriveTelemetry();
  public static VisionTelemetry vision = new VisionTelemetry();

  public static AlertLevel healthyRobot = AlertLevel.NONE;

  private Telemetry() {}

  public static void post() {
    test.post();
    drive.post();
    vision.post();

    SmartDashboard.putBoolean(PREFIX + "RobotHealth", healthyRobot == AlertLevel.NONE);
  }
}
