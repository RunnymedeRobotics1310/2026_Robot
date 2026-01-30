package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {

  // The LED strip is plugged into a DIO port on the RoboRIO
  private final AddressableLED ledStrip = new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);

  // Buffer of data to write to the LED strip
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(
      Constants.LightingConstants.LED_STRING_LENGTH);

  // Buffers
  public LightingSubsystem() {
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

}
