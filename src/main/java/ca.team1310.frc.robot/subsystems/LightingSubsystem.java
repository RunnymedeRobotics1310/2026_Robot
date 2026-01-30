package ca.team1310.frc.robot.subsystems;

import ca.team1310.frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

  public static Timer ledTimerOn = new Timer();
  public static Timer ledTimerOff = new Timer();

  private static final AddressableLED ledStrip =
      new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);
  private static final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  // Buffers
  public static final AddressableLEDBuffer ledTestingBufferOne =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  public static final AddressableLEDBuffer ledTestingBufferTwo =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  public static final AddressableLEDBuffer ledTestingBufferThree =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  // LED patterns
  public static final LEDPattern rainbowLedPattern = LEDPattern.rainbow(255, 128);
  public static final Distance kLedSpacing = Units.Meters.of(1 / 120.0);
  public final LEDPattern scrollingRainbowLedPattern =
      rainbowLedPattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), kLedSpacing);
  public static final LEDPattern yellowLEDPatern = LEDPattern.solid(Color.kYellow);
  public static final LEDPattern greenLedPattern = LEDPattern.solid(Color.kGreen);
  public static final LEDPattern whiteLedPattern = LEDPattern.solid(Color.kWhite);
  public static final LEDPattern purpleLedPattern = LEDPattern.solid(Color.kDarkViolet);
  public static final LEDPattern redLedPattern = LEDPattern.solid(Color.kRed);
  public static final LEDPattern blueLedPattern = LEDPattern.solid(Color.kBlue);
  public static final LEDPattern orangeLedPattern = LEDPattern.solid(Color.kOrange);

  public LightingSubsystem() {

    ledStrip.setLength(Constants.LightingConstants.LED_STRING_LENGTH);
    ledStrip.start();

    ledTimerOn.reset();
    ledTimerOn.start();
    ledTimerOff.reset();
  }

  // runs once every robot period
  public void periodic() {}

  public void setRangeColour(int rangeStart, int rangeEnd, Color color) {
    for (int i = rangeStart; i < rangeEnd; i++) {
      ledTestingBufferOne.setLED(i, color);
    }
    ledStrip.setData(ledTestingBufferOne);
  }

  public void setBufferPattern(AddressableLEDBuffer buffer, LEDPattern pattern) {
    pattern.applyTo(buffer);
    ledStrip.setData(buffer);
  }

  private void flashLed(AddressableLEDBuffer buffer, LEDPattern pattern) {
    if (ledTimerOn.get() >= .1) {
      ledTimerOn.reset();
      ledTimerOn.stop();
      ledTimerOff.start();

      pattern.applyTo(buffer);
      ledStrip.setData(buffer);
    } else if (ledTimerOff.get() >= .1) {
      ledTimerOff.reset();
      ledTimerOff.stop();
      ledTimerOn.start();

      LEDPattern.kOff.applyTo(buffer);
      ledStrip.setData(buffer);
    }
  }
}
