package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import static edu.wpi.first.wpilibj.util.Color.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.telemetry.Telemetry;

public class LightingSubsystem extends SubsystemBase {

  // The LED strip is plugged into a DIO port on the RoboRIO
  private final AddressableLED ledStrip =
      new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);

  // Buffer of data to write to the LED strip
  private final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);


  public static final LEDPattern rainbowLedPattern = LEDPattern.rainbow(255, 128);
  public static final Distance kLedSpacing = Meters.of(1 / 120.0);
  public final LEDPattern scrollingRainbowLedPattern =
      rainbowLedPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
  public static final LEDPattern yellowLEDPatern = LEDPattern.solid(kYellow);
  public static final LEDPattern greenLedPattern = LEDPattern.solid(kGreen);
  public static final LEDPattern whiteLedPattern = LEDPattern.solid(kWhite);
  public static final LEDPattern purpleLedPattern = LEDPattern.solid(kDarkViolet);
  public static final LEDPattern redLedPattern = LEDPattern.solid(kRed);
  public static final LEDPattern blueLedPattern = LEDPattern.solid(kBlue);
  public static final LEDPattern orangeLedPattern = LEDPattern.solid(kOrangeRed);
  private LEDPattern alliancePattern;

  // Blink Constants
  private final Timer blinkTimer = new Timer();
  private boolean isAllianceColor = false;

  public LightingSubsystem() {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.start();
    blinkTimer.start();
  }

  // runs once every robot period
  // TODO Make methods based on the current robot functions
  @Override
  public void periodic() {

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      alliancePattern = LEDPattern.solid(kFirstRed);
      } else alliancePattern = LEDPattern.solid(kFirstBlue);

    if (DriverStation.isEnabled()) {
      // if climbing
      // if aligned to climb
      // if climb is up
      // if ready to shoot
      // if in range
      // if intake running
      // else DEFAULT PATTERN (based on alliance?)
      if (/*Telemetry.climb.level == 3*/ false) {
        scrollingRainbowLedPattern.applyTo(ledBuffer);
      }
      else if (/*Telemetry.climb.level == 2*/ false) {
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, kViolet)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing)
                .applyTo(ledBuffer);
      }
      else if (/*Telemetry.climb.level == 1*/ false) {
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, kDarkViolet)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing)
                .applyTo(ledBuffer);
      }
      else if (/*Telemetry.climb.alignedToTower*/ false) {
        greenLedPattern.applyTo(ledBuffer);
      }
      else if (/*Telemetry.climb.climbEncoder > 0*/ false) {
        blink(LEDPattern.solid(kViolet), 0.25);
      }
      else if (/*Telemetry.shooter.atSpeed*/ false) {
        yellowLEDPatern.applyTo(ledBuffer);
      }
      else if (/*Telemetry.swerve.distanceToHub <= Constants.Swerve.MAX_SHOOTER_DIST*/ false) {
        orangeLedPattern.applyTo(ledBuffer);
      }
      else if (/*Telemetry.intake.intakeSpeed > 0*/ false) {
        blink(yellowLEDPatern, 0.5);
      }
      else {
        alliancePattern.applyTo(ledBuffer);
      }
    } else {
      if (/*Telemetry.climb.level == 3*/ false) {
        scrollingRainbowLedPattern.applyTo(ledBuffer);
      } else if (Telemetry.healthyRobot == Telemetry.AlertLevel.ERROR) {
        orangeLedPattern.blink(Second.of(0.1)).applyTo(ledBuffer);
      } else if (Telemetry.healthyRobot == Telemetry.AlertLevel.WARNING) {
        LEDPattern.solid(kGreenYellow).blink(Second.of(0.1)).applyTo(ledBuffer);
      } else if (Telemetry.swerve.hasVisPose) {
        greenLedPattern.applyTo(ledBuffer);
      } else {
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, kRed, kBlack)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(.1310), kLedSpacing)
                .applyTo(ledBuffer);
      }
    }

    // every loop, write the buffer to the LEDs
    ledStrip.setData(ledBuffer);
  }

  private void blink(LEDPattern pattern, double blinkPeriod) {
    if (blinkTimer.hasElapsed(blinkPeriod)) {
      blinkTimer.restart();
      isAllianceColor = !isAllianceColor;
    }

    LEDPattern active = isAllianceColor ? alliancePattern : pattern;
    active.applyTo(ledBuffer);
  }

}
