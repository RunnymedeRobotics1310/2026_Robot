package frc.robot.commands.auto.config;

import java.util.List;

public class AutoStep {

    public enum StepType {
        drive, rotate, shooter, intake, delay, parallel
    }

    public enum DriveMode {
        distance, time
    }

    public enum ShooterAction {
        on, on_with_duration, off
    }

    public enum IntakeAction {
        on, on_with_duration, off
    }

    public enum ParallelEndCondition {
        all, first
    }

    // Common
    public StepType type;

    // Drive fields
    public double direction;
    public double speedMPS;
    public DriveMode mode;
    public double distanceMetres;
    public double durationSeconds;
    public double headingDegrees;
    public double timeoutSeconds;

    // Rotate fields (headingDegrees and timeoutSeconds shared with drive)

    // Shooter fields
    public ShooterAction action;
    public double rpm;
    public double hoodPosition;
    public double kickerSpeed;
    public double kickerDelaySeconds;

    // Intake fields (action type reused via intakeAction)
    public IntakeAction intakeAction;
    public double speed;

    // Parallel fields
    public ParallelEndCondition endCondition;
    public List<AutoStep> commands;
}
