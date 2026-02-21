// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    // subsystem motors
    private final SparkMax bottomRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax topRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private double armSetpoint = 0;

    // TODO: fixme: we won't be using this. there will instead be 2 limit switches
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {

        led.setLength(60); // Set the length of the LED strip to match the buffer
        led.start();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // TODO Update telemetry

        beamEntry.setBoolean(isBeamBroken());

        if (isBeamBroken()) {
            setAllLEDs(255, 80, 0); // Orange if the beam is broken
        } else {
            setAllLEDs(20, 20, 20); // White if the beam is intact
        }

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
        armSetpoint = armSpeed;
    }

    public void stop() {
        setRollerSpeeds(0, 0);
        setArmSpeed(0);
        moveArmToAngle(0);
    }

    public double getArmAngle() {
        return 0;
    }

    public boolean moveArmToAngle(double armAngle) {
        /*
         * this method should be replaced with a setArmState(boolean extended) method.
         * we will only ever be moving the arm to 2 states, extended, or retracted.
         * there will be 2 limit switches, 1 for extended and 1 for retracted.
         */

        double currentAngle = getArmAngle();

        double angleError = armAngle - currentAngle;
        double desiredArmSpeed = Constants.IntakeConstants.ARM_FAST_SPEED;

        if (Math.abs(angleError) < Constants.IntakeConstants.ARM_ANGLE_TOLERANCE) {
            armSetpoint = 0;
            setArmSpeed(0);
            return true;
        }

        if (Math.abs(angleError) < Constants.IntakeConstants.ARM_SLOW_ZONE_ANGLE) {
            desiredArmSpeed = Constants.IntakeConstants.ARM_SLOW_ZONE_SPEED;
        } // dunno if this is needed

        if (angleError < 0) {
            desiredArmSpeed = -desiredArmSpeed;
        } // dont get this

        armSetpoint = desiredArmSpeed;
        setArmSpeed(desiredArmSpeed);
        return false;
    }//
     // Beam Break Sensor (if it looks weird im sorry im new to programming lol)

    private final DigitalInput beamBreak = new DigitalInput(0); // DIO port for the beam break sensor number 0
    private final AddressableLED led = new AddressableLED(0); // PWM port for the LED strip number 9
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60); // Buffer for the LED strip with 60
                                                                                 // LEDs

    // telemetry
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable telemetryTable = nt.getTable("telemetry");
    private final NetworkTableEntry beamEntry = telemetryTable.getEntry("beamBroken");
    private final NetworkTableEntry armAngleEntry = telemetryTable.getEntry("ledMode");

    // Set colour for LED
    public void setAllLEDs(int r, int g, int b) {
        for (int i = 0; i < 60; i++) {
            ledBuffer.setLED(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public boolean isBeamBroken() {
        return !beamBreak.get(); // Assuming the sensor returns false when the beam is broken

        // publish telemetry
        beamEntry.setBoolean(isBeamBroken());

        if (beamBroken) {
            setAllLEDs(255, 80, 0); // Orange if beam is broken
            ledEntry.setString("Orange");
        } else {
            setAllLEDs(20, 20, 20); // White if beam is intact
            ledEntry.setString("White");
        }

    }
    // setInterval(robotPeriodic,20); // Call robotPeriodic every 20 milliseconds
}
