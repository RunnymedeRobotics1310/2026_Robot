package frc.robot.util;

/**
 * Mutable XY vector.
 *
 * @author Quentin Field
 */
public class HubTargetVector {
  public double x;
  public double y;
  public double magnitude;
  public double angleDegrees;

  public HubTargetVector(double x, double y) {
    this.x = x;
    this.y = y;
    magnitude = Math.hypot(x, y);
    angleDegrees = Math.toDegrees(Math.atan2(y, x));
  }

  public void add(HubTargetVector v) {
    x += v.x;
    y += v.y;
    angleDegrees = (Math.toDegrees(Math.atan2(y, x)));
    magnitude = Math.hypot(x, y);
  }

  public void subtract(HubTargetVector v) {
    x -= v.x;
    y -= v.y;
    angleDegrees = (Math.toDegrees(Math.atan2(y, x)));
    magnitude = Math.hypot(x, y);
  }

  public void scale(double factor) {
    x *= factor;
    y *= factor;
    magnitude = Math.hypot(x, y);
  }
}
