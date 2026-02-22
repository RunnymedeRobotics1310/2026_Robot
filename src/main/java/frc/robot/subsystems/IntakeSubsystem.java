// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    //subsystem motors
    private final PWMSparkMax bottomRollerMotor = new PWMSparkMax(1);
//    private final SparkMax topRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
//    private final SparkMax armMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    //TODO: fixme: the arm motor will likely be controlled through pwm. this is how they are declared.
//    private final PWMSparkMax armMotor = new PWMSparkMax(ARM_MOTOR_PWM_PORT);

    private double armSetpoint = 0;

    //TODO: fixme: we won't be using this. there will instead be 2 limit switches
//    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    private final DigitalInput beamBreak = new DigitalInput(0); //DIO port for the beam break sensor number 0


    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // TODO Update telemetry
        Telemetry.intake.isHopperFull = isBeamBroken();
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
        }//dunno if this is needed

        if (angleError < 0) {
            desiredArmSpeed = -desiredArmSpeed;
        }//dont get this

        armSetpoint = desiredArmSpeed;
        setArmSpeed(desiredArmSpeed);
        return false;
    }//

    public boolean isBeamBroken() {
        return !beamBreak.get(); // Assuming the sensor returns false when the beam is broken
    }

}
