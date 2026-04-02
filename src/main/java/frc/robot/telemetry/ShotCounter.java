package frc.robot.telemetry;

import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class ShotCounter {

  private static final double DROP_THRESHOLD = 150.0;
  private static final double RECOVERY_THRESHOLD = 75.0;
  private static final double MIN_SPINNING_RPM = 1000.0;

  private enum State {
    READY,
    DIPPED
  }

  private State leftState = State.READY;
  private State rightState = State.READY;

  private int leftShotCount = 0;
  private int rightShotCount = 0;
  private int autoShotCount = 0;
  private int teleopShotCount = 0;
  private int pastHubShotCount = 0;

  private final NetworkTableEntry ntCountLeftShots;
  private final NetworkTableEntry ntCountRightShots;
  private final NetworkTableEntry ntCountTotalShots;
  private final NetworkTableEntry ntCountAutoShots;
  private final NetworkTableEntry ntCountTeleopShots;
  private final NetworkTableEntry ntCountPastHubShots;

  public ShotCounter() {
    NetworkTable table =
        NetworkTableInstance.getDefault().getTable("SmartDashboard/" + PREFIX + "Shooter");
    ntCountLeftShots = table.getEntry("CountLeftShots");
    ntCountRightShots = table.getEntry("CountRightShots");
    ntCountTotalShots = table.getEntry("CountTotalShots");
    ntCountAutoShots = table.getEntry("CountAutoShots");
    ntCountTeleopShots = table.getEntry("CountTeleopShots");
    ntCountPastHubShots = table.getEntry("CountPastHubShots");

    publish();
  }

  public void update(double targetRPM, double leftRPM, double rightRPM) {
    int leftShot = updateWheel(targetRPM, leftRPM, leftState, s -> leftState = s);
    int rightShot = updateWheel(targetRPM, rightRPM, rightState, s -> rightState = s);
    int newShots = leftShot + rightShot;

    // Only update all the counts and publish if we've detected a shot taken
    if (newShots > 0) {
      if (Telemetry.drive.pastHub) {
        pastHubShotCount += newShots;
      } else {
        leftShotCount += leftShot;
        rightShotCount += rightShot;

        if (DriverStation.isAutonomous()) {
          autoShotCount += newShots;
        } else if (DriverStation.isTeleop()) {
          teleopShotCount += newShots;
        }
      }

      publish();
    }
  }

  private int updateWheel(
      double target, double actual, State current, java.util.function.Consumer<State> setState) {
    if (target < MIN_SPINNING_RPM) {
      setState.accept(State.READY);
      return 0;
    }

    double error = target - actual;

    switch (current) {
      case READY:
        if (error > DROP_THRESHOLD && actual > MIN_SPINNING_RPM) {
          setState.accept(State.DIPPED);
        }
        return 0;

      case DIPPED:
        if (error <= RECOVERY_THRESHOLD) {
          setState.accept(State.READY);
          return 1;
        }
        return 0;

      default:
        return 0;
    }
  }

  /**
   * Set network tables entries for shots taken. This is not done in the usual Telemetry post()
   * style because this can be used for auto shot counting, and this should be updated as quickly as
   * is available
   */
  private void publish() {
    ntCountLeftShots.setInteger(leftShotCount);
    ntCountRightShots.setInteger(rightShotCount);
    ntCountTotalShots.setInteger(leftShotCount + rightShotCount);
    ntCountAutoShots.setInteger(autoShotCount);
    ntCountTeleopShots.setInteger(teleopShotCount);
    ntCountPastHubShots.setInteger(pastHubShotCount);
  }

  public void reset() {
    leftShotCount = 0;
    rightShotCount = 0;
    autoShotCount = 0;
    teleopShotCount = 0;
    pastHubShotCount = 0;
    leftState = State.READY;
    rightState = State.READY;

    publish();
  }

  public int getTeleopShotCount() {
    return teleopShotCount;
  }

  public int getAutoShotCount() {
    return autoShotCount;
  }

  public int getTotalShotCount() {
    return rightShotCount + leftShotCount;
  }
}
