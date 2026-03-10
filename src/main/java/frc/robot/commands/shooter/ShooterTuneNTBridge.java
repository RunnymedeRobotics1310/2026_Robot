package frc.robot.commands.shooter;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterTuneNTBridge {

  private final DoubleSubscriber targetRPMSub;
  private final DoubleSubscriber kickerSpeedSub;
  private final DoubleSubscriber hoodAngleSub;
  private final BooleanSubscriber shooterEnabledSub;
  private final BooleanSubscriber kickerEnabledSub;

  private final DoubleSubscriber heartbeatSub;

  private final DoublePublisher currentRPMPub;
  private final BooleanPublisher atSpeedPub;
  private final DoublePublisher distanceToHubPub;
  private final DoublePublisher angleToHubPub;

  private double lastHeartbeat = 0;
  private int staleCount = 0;
  private static final int STALE_THRESHOLD = 100; // ~2 seconds at 50Hz

  public ShooterTuneNTBridge() {
    NetworkTable table =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("1310")
            .getSubTable("shootertune");

    targetRPMSub = table.getDoubleTopic("targetRPM").subscribe(0);
    kickerSpeedSub = table.getDoubleTopic("kickerSpeed").subscribe(0);
    hoodAngleSub = table.getDoubleTopic("hoodAngle").subscribe(0);
    shooterEnabledSub = table.getBooleanTopic("shooterEnabled").subscribe(false);
    kickerEnabledSub = table.getBooleanTopic("kickerEnabled").subscribe(false);
    heartbeatSub = table.getDoubleTopic("heartbeat").subscribe(0);

    currentRPMPub = table.getDoubleTopic("currentRPM").publish();
    atSpeedPub = table.getBooleanTopic("atSpeed").publish();
    distanceToHubPub = table.getDoubleTopic("distanceToHub").publish();
    angleToHubPub = table.getDoubleTopic("angleToHub").publish();
  }

  public boolean isDashboardConnected() {
    double heartbeat = heartbeatSub.get();
    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      staleCount = 0;
      return true;
    }
    staleCount++;
    return staleCount < STALE_THRESHOLD;
  }

  public double getTargetRPM() {
    return targetRPMSub.get();
  }

  public double getKickerSpeed() {
    return kickerSpeedSub.get();
  }

  public double getHoodAngle() {
    return hoodAngleSub.get();
  }

  public boolean isShooterEnabled() {
    return shooterEnabledSub.get();
  }

  public boolean isKickerEnabled() {
    return kickerEnabledSub.get();
  }

  public void setCurrentRPM(double rpm) {
    currentRPMPub.set(rpm);
  }

  public void setAtSpeed(boolean atSpeed) {
    atSpeedPub.set(atSpeed);
  }

  public void setDistanceToHub(double distance) {
    distanceToHubPub.set(distance);
  }

  public void setAngleToHub(double angleDeg) {
    angleToHubPub.set(angleDeg);
  }
}
