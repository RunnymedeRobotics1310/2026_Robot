// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    //subsystem motors
    private final SparkMax bottomRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax topRollerMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    private double armSetpoint = 0;

    //bottom roller
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {
        SparkMaxConfig maxConfig = new SparkMaxConfig();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // TODO Update the smartdashboard
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setRollerSpeeds(double topLoraxSpeed, double bottomRollerSpeed) {
        topRollerMotor.set(topLoraxSpeed);
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
}



