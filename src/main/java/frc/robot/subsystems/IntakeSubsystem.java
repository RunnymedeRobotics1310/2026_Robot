// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    //subsystem motors
    //TODO: fixme: the motor will all be controlled through pwm. this is how they are declared.
    private final PWMSparkMax bottomRollerMotor = new PWMSparkMax(1);
//    private final SparkMax topRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
//    private final SparkMax armMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private final double armPosition = 1;
    private final double armStart = 0;
    private boolean armState = false;

    //TODO: fixme: we won't be using this. there will instead be 2 limit switches

    private final DigitalInput beamBreak = new DigitalInput(0); //DIO port for the beam break sensor number 0
    private final DigitalInput armExtended = new DigitalInput(0);
    private final DigitalInput armRetracted = new DigitalInput(0);
    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // TODO Update telemetry
        Telemetry.intake.isHopperFull = isBeamBroken();


        if (armState) {
            setArmToExtended(); //move motors forward
        } else {
            setArmToRetracted(); //move motors back
        }

        if (isArmExtended() && armState) {
            armStop(); //stop if arm reaches extension
        }
        if (isArmRetracted() && !armState) {
            armStop(); //stop if arm reaches retraction
        }

    }



    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setRollerSpeeds(double topRollerSpeed, double bottomRollerSpeed) {
//        topRollerMotor.set(topRollerSpeed);
        bottomRollerMotor.set(bottomRollerSpeed);
    }

    public void setArmSpeed(double armSpeed) {
//        armMotor.set(armSpeed);
    }

    public void rollerStop() {
        setRollerSpeeds(0, 0);
    }

    public void armStop() {
        setArmSpeed(0);
    }


    public boolean isArmExtended() {
        return armExtended.get(); //return true if arm extended
    }

    public boolean isArmRetracted(){
        return armRetracted.get(); //return true if arm retracted

    }

    public boolean getArmState(){
    return !isArmRetracted(); //return opposite of armRetracted
    }

    public void setArmToExtended(){
        setArmSpeed(0.05); //move to extended
    }

    public void setArmToRetracted(){
        setArmSpeed(-0.05); //move to retracted
    }


    public void setArmState(boolean extended) {

        armState = extended;

//        extended = getArmState();
//        if(!extended){
//            setArmToExtended();
//        }
//        else{
//            setArmToRetracted();
//        }

        /*
         * this method should be replaced with a setArmState(boolean extended) method.
         * we will only ever be moving the arm to 2 states, extended, or retracted.
         * there will be 2 limit switches, 1 for extended and 1 for retracted.
         */

    }
public void setRollers(boolean roll){

        if(roll){
            setRollerSpeeds(1,-1);
        }
}


    public boolean isBeamBroken() {
        return !beamBreak.get(); // Assuming the sensor returns false when the beam is broken
    }

}
