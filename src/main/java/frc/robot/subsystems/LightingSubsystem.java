package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.util.Color.kDarkViolet;
import static edu.wpi.first.wpilibj.util.Color.kFirstBlue;
import static edu.wpi.first.wpilibj.util.Color.kFirstRed;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static edu.wpi.first.wpilibj.util.Color.kGreenYellow;
import static edu.wpi.first.wpilibj.util.Color.kOrangeRed;
import static edu.wpi.first.wpilibj.util.Color.kPink;
import static edu.wpi.first.wpilibj.util.Color.kRed;
import static edu.wpi.first.wpilibj.util.Color.kViolet;
import static edu.wpi.first.wpilibj.util.Color.kYellow;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.telemetry.Telemetry.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.telemetry.Telemetry.*;

public class LightingSubsystem extends SubsystemBase {

  // The LED strip is plugged into a DIO port on the RoboRIO
  private final AddressableLED ledStrip = new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);

  // Buffer of data to write to the LED strip
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(
      Constants.LightingConstants.LED_STRING_LENGTH);

  public static final LEDPattern rainbowLedPattern = LEDPattern.rainbow(255, 128);
  public static final Distance kLedSpacing = Meters.of(1 / 120.0);
  public final LEDPattern scrollingRainbowLedPattern = rainbowLedPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5),
      kLedSpacing);
  public static final LEDPattern yellowLEDPatern = LEDPattern.solid(kYellow);
  public static final LEDPattern greenLedPattern = LEDPattern.solid(kGreen);
  public static final LEDPattern purpleLedPattern = LEDPattern.solid(kDarkViolet);
  public static final LEDPattern orangeLedPattern = LEDPattern.solid(kOrangeRed);
  public static final LEDPattern pinkLedPattern = LEDPattern.solid(kPink);

  private LEDPattern alliancePattern;

  // Blink Constants
  private final Timer blinkTimer = new Timer();
  private int visPoseCount = 0;
  private int shooterCount = 0;
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
    } else
      alliancePattern = LEDPattern.solid(kFirstBlue);

    if (DriverStation.isEnabled()) {
      if (shooter.targetShooterRPM > 0) {
      if (Math.abs(shooter.targetShooterRPM - shooter.currentShooterRPM) < ACCPETED_SHOOTER_ERROR)
        shooterCount = 10;
      } else {
        shooterCount = 0;
      }
      // if climbing
      // if aligned to climb
      // if climb is up
      // if intake running
      // if ready to shoot
      // if in range
      // else DEFAULT PATTERN (based on alliance?)

      if (/* Telemetry.climb.level == 3 */ false) {
        scrollingRainbowLedPattern.applyTo(ledBuffer);

      } else if (/* Telemetry.climb.level == 2 */ false) {
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, kViolet)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing)
            .applyTo(ledBuffer);

      } else if (/* Telemetry.climb.level == 1 */ false) {
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, kDarkViolet)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing)
            .applyTo(ledBuffer);

      } else if (/* Telemetry.climb.alignedToTower */ false) {
        greenLedPattern.applyTo(ledBuffer);

      } else if (/* Telemetry.climb.climbEncoder > 0 */ false) {
        blink(LEDPattern.solid(kViolet), 0.25);

      } else if (intake.topRollerSpeed > 0) {
        blink(yellowLEDPatern, 0.5);

      } else if (shooterCount > 0) { // shooterAtSpeed
        shooterCount--;
        yellowLEDPatern.applyTo(ledBuffer);

      } else if (drive.distanceToHub <= SUPER_FAR_SHOOTING_DISTANCE && false) {
        orangeLedPattern.applyTo(ledBuffer);

      } else {
        alliancePattern.applyTo(ledBuffer);

      }
    } else { // disabled

      if (swerve.hasVisPose) visPoseCount = 10;

      if (/* Telemetry.climb.level == 3 */ false) {
        scrollingRainbowLedPattern.applyTo(ledBuffer);

      } else if (healthyRobot == AlertLevel.ERROR) {
        orangeLedPattern.blink(Second.of(0.1)).applyTo(ledBuffer);

      } else if (healthyRobot == AlertLevel.WARNING) {
        LEDPattern.solid(kGreenYellow).blink(Second.of(0.1)).applyTo(ledBuffer);

      } else if (visPoseCount > 0) { // hasVisPose
        visPoseCount--;
        greenLedPattern.applyTo(ledBuffer);

      } else {
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, kRed, new Color(15, 0, 0))
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
