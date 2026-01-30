package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.XboxController;

public class GameController extends XboxController {

  public GameController(int port) {
    super(port);
  }

  private static final double DEADBAND = 0.2;
  private static final double SLOW_X = 0.7;
  private static final double SLOW_Y = .4;

  // Calculate the slope and intercept for each of the
  // slow zone and fast zone line segments.
  private static final double SLOW_M = SLOW_Y / (SLOW_X - DEADBAND);
  private static final double SLOW_B = -SLOW_M * DEADBAND;

  private static final double FAST_M = (1.0 - SLOW_Y) / (1.0 - SLOW_X);
  private static final double FAST_B = -(FAST_M * SLOW_X) + SLOW_Y;

  @Override
  public double getRawAxis(int axis) {

    double rawAxisValue = super.getRawAxis(axis);

    // Don't scale the triggers
    if (axis == Axis.kRightTrigger.value || axis == Axis.kLeftTrigger.value) {
      return rawAxisValue;
    }

    // Deadband the axis
    if (Math.abs(rawAxisValue) < DEADBAND) {
      return 0.0;
    }

    // For Y axis values, invert the value so that forward is positive
    if (axis == Axis.kLeftY.value || axis == Axis.kRightY.value) {
      rawAxisValue = -rawAxisValue;
    }

    // Shape the output of the axis to have a slow slope below the transition
    // point and a sharper slope after
    // y = mx + b
    if (Math.abs(rawAxisValue) < SLOW_X) {
      return (SLOW_M * Math.abs(rawAxisValue) + SLOW_B) * Math.signum(rawAxisValue);
    }

    return (FAST_M * Math.abs(rawAxisValue) + FAST_B) * Math.signum(rawAxisValue);
  }

  @Override
  public String toString() {

    StringBuilder sb = new StringBuilder();

    /*
     * Axis
     */
    // Left stick
    sb.append('(')
        .append(Math.round(getLeftX() * 100d) / 100d)
        .append(',')
        .append(Math.round(getLeftY() * 100d) / 100d)
        .append(')');

    // Right stick
    sb.append('(')
        .append(Math.round(getRightX() * 100d) / 100d)
        .append(',')
        .append(Math.round(getRightY() * 100d) / 100d)
        .append(')');

    // Triggers
    sb.append('[')
        .append(Math.round(getLeftTriggerAxis() * 100d) / 100d)
        .append(',')
        .append(Math.round(getRightTriggerAxis() * 100d) / 100d)
        .append("] ");

    /*
     * POV
     */
    if (getPOV() >= 0) {
      sb.append("POV(").append(getPOV()).append(") ");
    }

    /*
     * Buttons
     */
    if (getLeftBumperButton()) {
      sb.append("LB ");
    }
    if (getRightBumperButton()) {
      sb.append("RB ");
    }
    if (getAButton()) {
      sb.append("A ");
    }
    if (getBButton()) {
      sb.append("B ");
    }
    if (getXButton()) {
      sb.append("X ");
    }
    if (getYButton()) {
      sb.append("Y ");
    }
    if (getBackButton()) {
      sb.append("Back ");
    }
    if (getBackButton()) {
      sb.append("Start ");
    }
    if (getLeftStickButtonPressed()) {
      sb.append("LStick ");
    }
    if (getRightStickButtonPressed()) {
      sb.append("RStick ");
    }

    return sb.toString();
  }
}
