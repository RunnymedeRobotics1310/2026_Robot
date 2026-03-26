package frc.robot.telemetry;

/**
 * Immutable record capturing per-module telemetry data for swerve drive debugging.
 *
 * @param moduleName the name of the swerve module (e.g., "frontleft")
 * @param commandedAngleDeg the commanded angle in degrees
 * @param actualAngleDeg the actual measured angle in degrees
 * @param angleErrorDeg the difference between commanded and actual angle
 * @param commandedVelocityMPS the commanded velocity in meters per second
 * @param actualVelocityMPS the actual measured velocity in meters per second
 * @param velocityErrorMPS the difference between commanded and actual velocity
 */
public record ModuleTelemetry(
    String moduleName,
    double commandedAngleDeg,
    double actualAngleDeg,
    double angleErrorDeg,
    double commandedVelocityMPS,
    double actualVelocityMPS,
    double velocityErrorMPS) {

  /**
   * Create a ModuleTelemetry with computed error values.
   *
   * @param moduleName the name of the swerve module
   * @param commandedAngleDeg the commanded angle in degrees
   * @param actualAngleDeg the actual measured angle in degrees
   * @param commandedVelocityMPS the commanded velocity in meters per second
   * @param actualVelocityMPS the actual measured velocity in meters per second
   * @return a new ModuleTelemetry with errors computed
   */
  public static ModuleTelemetry create(
      String moduleName,
      double commandedAngleDeg,
      double actualAngleDeg,
      double commandedVelocityMPS,
      double actualVelocityMPS) {
    double angleError = normalizeAngle(commandedAngleDeg - actualAngleDeg);
    double velocityError = commandedVelocityMPS - actualVelocityMPS;
    return new ModuleTelemetry(
        moduleName,
        commandedAngleDeg,
        actualAngleDeg,
        angleError,
        commandedVelocityMPS,
        actualVelocityMPS,
        velocityError);
  }

  /** Normalize angle to be within -180 to 180 degrees. */
  private static double normalizeAngle(double angleDeg) {
    while (angleDeg > 180) angleDeg -= 360;
    while (angleDeg < -180) angleDeg += 360;
    return angleDeg;
  }
}
