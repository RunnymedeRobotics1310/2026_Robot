package frc.robot.telemetry;

/**
 * Detects wheel slip by comparing gyro-measured rotation rate to wheel-calculated rotation rate.
 * When the robot's wheels slip, the wheel-calculated rotation will differ significantly from what
 * the gyro measures.
 */
public class SlipDetector {

  /** Threshold for detecting slip (as a fraction, e.g., 0.10 = 10% difference). */
  private static final double SLIP_THRESHOLD = 0.10;

  /** Minimum rotation rate to consider for slip detection (rad/s). Avoids noise at low speeds. */
  private static final double MIN_ROTATION_RATE = 0.1;

  private boolean slipDetected = false;
  private double gyroVsWheelDelta = 0.0;
  private double gyroRotationRateRadPS = 0.0;
  private double wheelRotationRateRadPS = 0.0;

  /**
   * Update slip detection with new rotation rate data.
   *
   * @param gyroRotationRateRadPS the rotation rate from the gyro in radians per second
   * @param wheelRotationRateRadPS the rotation rate calculated from wheel velocities in radians per
   *     second
   */
  public void update(double gyroRotationRateRadPS, double wheelRotationRateRadPS) {
    this.gyroRotationRateRadPS = gyroRotationRateRadPS;
    this.wheelRotationRateRadPS = wheelRotationRateRadPS;
    this.gyroVsWheelDelta = gyroRotationRateRadPS - wheelRotationRateRadPS;

    // Only detect slip when robot is actually rotating meaningfully
    double absGyro = Math.abs(gyroRotationRateRadPS);
    double absWheel = Math.abs(wheelRotationRateRadPS);
    double maxRate = Math.max(absGyro, absWheel);

    if (maxRate < MIN_ROTATION_RATE) {
      slipDetected = false;
      return;
    }

    // Calculate relative difference
    double relativeDiff = Math.abs(gyroVsWheelDelta) / maxRate;
    slipDetected = relativeDiff > SLIP_THRESHOLD;
  }

  /**
   * @return true if slip was detected in the last update
   */
  public boolean isSlipDetected() {
    return slipDetected;
  }

  /**
   * @return the difference between gyro and wheel rotation rates (rad/s)
   */
  public double getGyroVsWheelDelta() {
    return gyroVsWheelDelta;
  }

  /**
   * @return the last gyro rotation rate (rad/s)
   */
  public double getGyroRotationRateRadPS() {
    return gyroRotationRateRadPS;
  }

  /**
   * @return the last wheel-calculated rotation rate (rad/s)
   */
  public double getWheelRotationRateRadPS() {
    return wheelRotationRateRadPS;
  }

  /** Reset the slip detector state. */
  public void reset() {
    slipDetected = false;
    gyroVsWheelDelta = 0.0;
    gyroRotationRateRadPS = 0.0;
    wheelRotationRateRadPS = 0.0;
  }
}
