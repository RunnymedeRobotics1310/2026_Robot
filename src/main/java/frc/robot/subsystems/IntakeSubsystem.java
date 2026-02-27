package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

    //subsystem motors
    private final PWMSparkMax bottomRollerMotor = new PWMSparkMax(BOTTOM_ROLLER_PWM_PORT);
    private final PWMSparkMax topRollerMotor = new PWMSparkMax(TOP_ROLLER_PWM_PORT);
    private final PWMSparkMax doorMotor = new PWMSparkMax(DOOR_PWM_PORT);

    private boolean doorState = false;

    //TODO: fixme: use a timer instead of limit switches

    private final DigitalInput beamBreak = new DigitalInput(-1); //DIO port for the beam break sensor number 0
    private final DigitalInput doorExtended = new DigitalInput(-1);
    private final DigitalInput doorRetracted = new DigitalInput(-1);
    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // TODO Update telemetry
        Telemetry.intake.isHopperFull = isBeamBroken();

        if (doorState) {
            setDoorToExtended(); //move motors forward
        } else {
            setDoorToRetracted(); //move motors back
        }

        if (isDoorExtended() && doorState) {
            doorStop(); //stop if door reaches extension
        }
        if (isDoorRetracted() && !doorState) {
            doorStop(); //stop if door reaches retraction
        }
    }

    public void setRollerSpeeds(double topRollerSpeed, double bottomRollerSpeed) {
        topRollerMotor.set(topRollerSpeed);
        bottomRollerMotor.set(bottomRollerSpeed);
    }

    public void setDoorSpeed(double doorSpeed) {
        doorMotor.set(doorSpeed);
    }

    public void rollerStop() {
        setRollerSpeeds(0, 0);
    }

    public void doorStop() {
        setDoorSpeed(0);
    }

    public boolean isDoorExtended() {
        return doorExtended.get(); //return true if door extended
    }

    public boolean isDoorRetracted(){
        return doorRetracted.get(); //return true if door retracted
    }

    public boolean getDoorState(){
        return !isDoorRetracted(); //return opposite of doorRetracted
    }

    public void setDoorToExtended(){
        setDoorSpeed(DOOR_SPEED); //move to extended
    }

    public void setDoorToRetracted(){
        setDoorSpeed(-DOOR_SPEED); //move to retracted
    }

    public void setDoorState(boolean extended) {
        doorState = extended;
    }

    public void setRollers(boolean roll) {
        if (roll) {
            setRollerSpeeds(INTAKE_SPEED,-INTAKE_SPEED);
        }
    }

    public boolean isBeamBroken() {
        return !beamBreak.get(); // Assuming the sensor returns false when the beam is broken
    }

}
