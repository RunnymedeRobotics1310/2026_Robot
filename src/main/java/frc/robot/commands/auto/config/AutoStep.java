package frc.robot.commands.auto.config;

import java.util.List;

public class AutoStep {

    public enum StepType {
        drive, rotate, shooter, intake, delay, parallel,
        face_target, vision_approach_tag, hold
    }

    public enum DriveMode {
        distance, time, to_pose, velocity
    }

    public enum VelocityFrame {
        field, robot
    }

    public enum FaceTargetType {
        hub, point
    }

    public enum ShooterAction {
        on, on_with_duration, off
    }

    public enum IntakeAction {
        on, on_with_duration, off
    }

    public enum ParallelEndCondition {
        all, first, deadline
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
    public int deadlineIndex;
    public List<AutoStep> commands;

    // Drive-to-pose fields
    public double xMetres;
    public double yMetres;
    public double positionToleranceMetres;
    public double headingToleranceDegrees;

    // Velocity drive fields
    public VelocityFrame frame;
    public double vxMPS;
    public double vyMPS;

    // Face target fields
    public FaceTargetType target;
    public double targetXMetres;
    public double targetYMetres;

    // Vision approach fields
    public boolean rightSide;
}
