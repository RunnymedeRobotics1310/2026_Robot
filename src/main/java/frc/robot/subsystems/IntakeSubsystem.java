package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

  // subsystem motors
  private final PWMSparkMax bottomRollerMotor = new PWMSparkMax(BOTTOM_ROLLER_PWM_PORT);
  private final PWMSparkMax topRollerMotor = new PWMSparkMax(TOP_ROLLER_PWM_PORT);
  private final SparkMax doorMotor = new SparkMax(DOOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

  private final DigitalInput doorClosedLimit = new DigitalInput(DOOR_CLOSED_LIMIT_DIO_PORT);

  private boolean doorState = false;
  private double doorSetpoint = 0;

  //  private final DigitalInput beamBreak = new DigitalInput(-1);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO Update telemetry
    Telemetry.intake.isHopperFull = isBeamBroken();
    Telemetry.intake.doorSetpoint = doorSetpoint;
    Telemetry.intake.doorAngle = getDoorAngle();
    Telemetry.intake.isDoorClosed = getDoorClosed();

    updateDoorSpeed();

    if (getDoorClosed()) {
      doorMotor.getEncoder().setPosition(0);
    }
  }

  public void setRollerSpeeds(double topRollerSpeed, double bottomRollerSpeed) {
    topRollerMotor.set(topRollerSpeed);
    bottomRollerMotor.set(bottomRollerSpeed);
  }

  public void setDoorSpeed(double doorSpeed) {
    doorMotor.set(doorSpeed);
  }

  public double getDoorAngle() {
    return doorMotor.getEncoder().getPosition() * DOOR_ENCODERS_TO_DEGREES;
  }

  public void setDoorState(boolean extended) {
    doorState = extended;
  }

  public void setDoorSetpoint(double setpoint) {
    doorSetpoint = setpoint;
  }

  public double getDoorSetpoint() {
    return doorSetpoint;
  }

  public boolean getDoorClosed() {
    return !doorClosedLimit.get();
  }

  public boolean getDoorState() {
    return doorState;
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

  private void updateDoorSpeed() {
    double error = doorSetpoint - getDoorAngle();

    setDoorSpeed(error * DOOR_KP);

    if (doorSetpoint == 0 && !getDoorClosed()) {
      setDoorSpeed(-0.1);
    }
  }

  public void stop() {
    setRollerSpeeds(0, 0);
    setDoorSpeed(0);
    setDoorState(false);
  }
}
