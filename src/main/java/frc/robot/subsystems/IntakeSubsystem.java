// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    //subsystem motors
    private final SparkMax bottomRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax topRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private final double armPosition = 1;
    private final double armStart = 0;

    private double armSetpoint = 0; //no?

    //TODO: fixme: we won't be using this. there will instead be 2 limit switches
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // TODO Update telemetry
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setRollerSpeeds(double topRollerSpeed, double bottomRollerSpeed) {
        topRollerMotor.set(topRollerSpeed);
        bottomRollerMotor.set(bottomRollerSpeed);
    }

    public void setArmSpeed(double armSpeed) {
        armMotor.set(armSpeed);
    }

    public void stop() {
        setRollerSpeeds(0, 0);
        setArmSpeed(0);
        setArmState(false);
    }

    public double getArmAngle() {
        return 0;
    } //prob remove

    public void armExtend (){
        setArmSpeed(0.5);
    }//extends arm

    public void armRetract (){
        setArmSpeed(-0.5);
    }//retracts arm

    public boolean setArmState(boolean extended) {
        /*
         * this method should be replaced with a setArmState(boolean extended) method.
         * we will only ever be moving the arm to 2 states, extended, or retracted.
         * there will be 2 limit switches, 1 for extended and 1 for retracted.
         */
        if (extended) {
            armExtend();
        } else {
            armRetract();
        }
return true;
    }

    //set arm extended movement
    //check if the motor is at position
    //move motor
    //if the motor extends too far, stop

    //set arm retracted movement
    //check if the motor is at position
    //move motor backwards
    //if the motor retracts too far, stop
}



