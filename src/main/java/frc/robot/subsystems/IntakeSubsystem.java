package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

  // subsystem motors
  private final PWMSparkMax bottomRollerMotor = new PWMSparkMax(BOTTOM_ROLLER_PWM_PORT);
  private final PWMSparkMax topRollerMotor = new PWMSparkMax(TOP_ROLLER_PWM_PORT);
  private final PWMSparkMax doorMotor = new PWMSparkMax(DOOR_PWM_PORT);

  private final Timer ravenTimer = new Timer();
  private boolean doorState = false;

  //  private final DigitalInput beamBreak = new DigitalInput(-1);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO Update telemetry
    Telemetry.intake.isHopperFull = isBeamBroken();

    if (!ravenTimer.hasElapsed(DOOR_MOVE_TIME)) {

      //      if (doorState) {
      //        setDoorToExtended();
      //      } else {
      //        setDoorToRetracted();
      //      }

    } else {
      //      doorStop();
      //      ravenTimer.stop();
      //      ravenTimer.reset();
    }
  }

  public void setRollerSpeeds(double topRollerSpeed, double bottomRollerSpeed) {
    topRollerMotor.set(topRollerSpeed);
    bottomRollerMotor.set(bottomRollerSpeed);
  }

  public void setDoorSpeed(double doorSpeed) {
    doorMotor.set(doorSpeed);
  }

  public void setDoorState(boolean extended) {
    if (doorState != extended) {
      doorState = extended;
      ravenTimer.reset();
      ravenTimer.start();
    }
  }

  public boolean getDoorState() {
    return doorState;
  }

  public void setDoorToExtended() {
    setDoorSpeed(DOOR_SPEED) // move to extended
    ;
  }

  public void setDoorToRetracted() {
    setDoorSpeed(-DOOR_SPEED); // move to retracted
  }

  public void setRollers(boolean roll) {
    if (roll) {
      setRollerSpeeds(INTAKE_SPEED, INTAKE_SPEED);
    }
  }

  public boolean isBeamBroken() {
    //      return !beamBreak.get(); // Assuming the sensor returns false when the beam is broken
    return false;
  }

  public void rollerStop() {
    setRollerSpeeds(0, 0);
  }

  public void doorStop() {
    setDoorSpeed(0);
  }

  public void stop() {
    setRollerSpeeds(0, 0);
    setDoorSpeed(0);
  }
}
