package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {

  // The LED strip is plugged into a DIO port on the RoboRIO
  private final AddressableLED ledStrip = new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);

  // Buffer of data to write to the LED strip
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(
      Constants.LightingConstants.LED_STRING_LENGTH - 1);

  private final AddressableLEDBufferView climbLedView = ledBuffer.createView(
      Constants.LightingConstants.LED_CLIMB_VIEW_START,
      Constants.LightingConstants.LED_CLIMB_VIEW_END);
  private final AddressableLEDBufferView intakeLedView = ledBuffer.createView(
      Constants.LightingConstants.LED_INTAKE_VIEW_START,
      Constants.LightingConstants.LED_INTAKE_VIEW_END);
  private final AddressableLEDBufferView robotHealthLedView = ledBuffer.createView(
      Constants.LightingConstants.LED_ROBOT_HEALTH_VIEW_START,
      Constants.LightingConstants.LED_ROBOT_HEALTH_VIEW_END);
  private final AddressableLEDBufferView driveLedView = ledBuffer.createView(
      Constants.LightingConstants.LED_DRIVE_VIEW_START,
      Constants.LightingConstants.LED_DRIVE_VIEW_END);

  public static final LEDPattern rainbowLedPattern = LEDPattern.rainbow(255, 128);
  // FIXME check the spacing - could be 60 LEDs/m - not sure why we need this?
  public static final Distance kLedSpacing = Units.Meters.of(1 / 120.0);
  public final LEDPattern scrollingRainbowLedPattern = rainbowLedPattern
      .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), kLedSpacing);
  public static final LEDPattern yellowLEDPatern = LEDPattern.solid(Color.kYellow);
  public static final LEDPattern greenLedPattern = LEDPattern.solid(Color.kGreen);
  public static final LEDPattern whiteLedPattern = LEDPattern.solid(Color.kWhite);
  public static final LEDPattern purpleLedPattern = LEDPattern.solid(Color.kDarkViolet);
  public static final LEDPattern redLedPattern = LEDPattern.solid(Color.kRed);
  public static final LEDPattern blueLedPattern = LEDPattern.solid(Color.kBlue);
  public static final LEDPattern orangeLedPattern = LEDPattern.solid(Color.kOrange);

  public LightingSubsystem() {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.start();
  }

  // runs once every robot period
  @Override
  public void periodic() {
    // every loop, write the buffer to the LEDs
    ledStrip.setData(ledBuffer);
  }

  // TODO Make methods based on the current robot functions
  // to set data in the LEDbuffer based on some actions
  public void setRobotFunction() {
  }

  public void setRobotClimbState(ClimbStates climbState) {
    switch (climbState) {
      case GOING_TO_CLIMB:
        yellowLEDPatern.applyTo(climbLedView);
        break;

      case CLIMBING:
        greenLedPattern.applyTo(climbLedView);
        break;

      case CLIMB_FAILED:
        redLedPattern.applyTo(climbLedView);
        break;

      case UNUSED_STATE1:
        break;

      case UNUSED_STATE2:
        break;

      case UNUSED_STATE3:
        break;
    }
  }

  public void setRobotIntakeState(IntakeStates intakeState) {
    switch (intakeState) {
      case FEEDING_INTAKE:
        yellowLEDPatern.applyTo(climbLedView);

        break;

      case SHOOTING:
        greenLedPattern.applyTo(climbLedView);
        break;

      case HOPPER_FULL:
        redLedPattern.applyTo(climbLedView);
        break;

      case UNUSED_STATE1:
        break;

      case UNUSED_STATE2:
        break;

      case UNUSED_STATE3:
        break;
    }
  }

  public void setRobotHealthState(RobotHealthStates healthState) {
    switch (healthState) {
      case ROBOT_GOOD:
        greenLedPattern.applyTo(climbLedView);

        break;

      case ROBOT_OKAY:
        yellowLEDPatern.applyTo(climbLedView);
        break;

      case ROBOT_BAD:
        redLedPattern.applyTo(climbLedView);
        break;

      case UNUSED_STATE1:
        break;

      case UNUSED_STATE2:
        break;

      case UNUSED_STATE3:
        break;
    }
  }

  public void setRobotDriveState(DriveStates driveState) {

    switch (driveState) {
      case NORMAL_DRIVING:
        greenLedPattern.applyTo(climbLedView);
        break;

      case BOOST:
        purpleLedPattern.applyTo(climbLedView);
        break;

      case ALLIANCE_BLUE:
        blueLedPattern.applyTo(climbLedView);
        break;

      case UNUSED_RED:
        redLedPattern.applyTo(climbLedView);
        break;

      case UNUSED_STATE2:
        break;

      case UNUSED_STATE3:
        break;
    }
  }

  // Robot states
  public enum ClimbStates {
    GOING_TO_CLIMB,
    CLIMBING,
    CLIMB_FAILED,
    UNUSED_STATE1,
    UNUSED_STATE2,
    UNUSED_STATE3
  }

  public enum DriveStates {
    NORMAL_DRIVING,
    BOOST,
    ALLIANCE_BLUE,
    UNUSED_RED,
    UNUSED_STATE2,
    UNUSED_STATE3
  }

  public enum RobotHealthStates {
    ROBOT_GOOD,
    ROBOT_OKAY,
    ROBOT_BAD,
    UNUSED_STATE1,
    UNUSED_STATE2,
    UNUSED_STATE3
  }

  public enum IntakeStates {
    FEEDING_INTAKE,
    SHOOTING,
    HOPPER_FULL,
    UNUSED_STATE1,
    UNUSED_STATE2,
    UNUSED_STATE3
  }
}
