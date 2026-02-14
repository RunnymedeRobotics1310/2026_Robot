package frc.robot.commands.swerve;

import static ca.team1310.swerve.utils.SwerveUtils.normalizeDegrees;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.Swerve.ROTATION_CONFIG;
import static frc.robot.Constants.Swerve.TRANSLATION_CONFIG;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;
import static frc.robot.operatorInput.OperatorInput.Axis.X;
import static frc.robot.operatorInput.OperatorInput.Axis.Y;
import static frc.robot.operatorInput.OperatorInput.Stick.LEFT;
import static frc.robot.operatorInput.OperatorInput.Stick.RIGHT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.operatorInput.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class TeleopDriveCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final OperatorInput oi;
    private final LimelightVisionSubsystem vision;
    private boolean invert;
    private Double headingSetpointDeg = null;
    private boolean fieldOriented = true;
    private Timer rotationSettleTimer = new Timer();
    private boolean prevRotate180Val = false;
    private boolean lockOnHub = false;

    /** Used to drive a swerve robot in full field-centric mode. */
    public TeleopDriveCommand(
            SwerveSubsystem swerve,
            LimelightVisionSubsystem vision,
            OperatorInput operatorInput) {
        this.swerve = swerve;
        this.vision = vision;
        this.oi = operatorInput;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        logCommandStart();
        rotationSettleTimer.start();
        rotationSettleTimer.reset();
        headingSetpointDeg = null;

        // The FRC field-oriented coordinate system
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        final Alliance alliance = getRunnymedeAlliance();
        SmartDashboard.putString("1310/Alliance", alliance.toString());

        // The coordinate system defines (0,0) as the right side of the blue alliance wall. The
        // x-axis is positive toward the red alliance, and the y-axis is positive to the left.
        // When the robot is on the red alliance, we need to invert inputs for the stick values
        // to move the robot in the right direction.
        this.invert = (alliance == Alliance.Red);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // @Override
    @Override
    public void execute() {
        final boolean isZeroGyro = oi.isZeroGyro();

        // With the driver standing behind the driver station glass, "forward" on the left stick is
        // its y value, but that should convert into positive x movement on the field. The
        // Runnymede Controller inverts stick y-axis values, so "forward" on stick is positive.
        // Thus, positive y stick axis maps to positive x translation on the field.
        final double vX = oi.getDriverControllerAxis(LEFT, Y);

        // Left and right movement on the left stick (the stick's x-axis) maps to the y-axis on the
        // field. Left on the stick (negative x) maps to positive y on the field, and vice versa.
        // Thus, negative x stick axis maps to positive y translation on the field.
        final double vY = -oi.getDriverControllerAxis(LEFT, X);

        // Left and right on the right stick will change the direction the robot is facing - its
        // heading. Positive x values on the stick translate to clockwise motion, and vice versa.
        // The coordinate system has positive motion as CCW.
        // Therefore, negative x stick value maps to positive rotation on the field.
        final double ccwRotAngularVelPct =
                -oi.getDriverControllerAxis(RIGHT, X) * 0.25; // TODO: put this in constants?

        final boolean rotate180Val = oi.getRotate180Val();

        final boolean faceHub = oi.getFaceHub();

        // Compute boost factor
        final boolean isSlow = oi.isSlowMode();
        //    final boolean isSlow = false;
        final boolean isFast = oi.isFastMode();
        final double boostFactor =
                isSlow ? SLOW_SPEED_FACTOR : (isFast ? MAX_SPEED_FACTOR : GENERAL_SPEED_FACTOR);

        Translation2d velocity = calculateTeleopVelocity(vX, vY, boostFactor, invert);

        final boolean doFlip = rotate180Val && !prevRotate180Val;
        prevRotate180Val = rotate180Val;

        final double omegaRadiansPerSecond;
        double desiredOmegaRadiansPerSecond;
        double correctedCcwRotAngularVelPct = ccwRotAngularVelPct;

        // Compute Omega
        if (correctedCcwRotAngularVelPct != 0) {
            // User is steering!
            lockOnHub = false;
            omegaRadiansPerSecond =
                    Math.pow(correctedCcwRotAngularVelPct, 3) * ROTATION_CONFIG.maxRotVelocityRadPS();
            // Save previous heading for when we are finished steering and slow enough.
            // headingSetpoint = Rotation2d.fromDegrees(swerve.getYaw());
            headingSetpointDeg = null;
            rotationSettleTimer.reset();
        } else {
            // Translating only. Just drive on robot yaw
            // TODO: tune timer duration
            if (rotationSettleTimer.hasElapsed(0.5) && headingSetpointDeg == null) {
                headingSetpointDeg = swerve.getYaw();
            }

            if (faceHub || lockOnHub) {
                lockOnHub =  true;
                headingSetpointDeg = swerve.angleToHub().getDegrees();
            }

            // rotate 180º button
            if (doFlip) {
                lockOnHub = false;
                if (headingSetpointDeg == null) {
                    headingSetpointDeg = swerve.getYaw() + 180;
                } else {
                    headingSetpointDeg += 180;
                }
            }

            // Don't spin around on zero gyro!
            if (isZeroGyro) {
                lockOnHub = false;
                headingSetpointDeg = null;
            }

            // Set omega
            if (headingSetpointDeg == null
                    || (velocity.getNorm() < 0.05 && Math.abs(headingSetpointDeg - swerve.getYaw()) < 2)) {
                omegaRadiansPerSecond = 0;
            } else {
                headingSetpointDeg = normalizeDegrees(headingSetpointDeg);
                omegaRadiansPerSecond =
                        swerve.computeOmega(headingSetpointDeg, ROTATION_CONFIG.maxRotVelocityRadPS());
            }
        }

        if (fieldOriented) {
            // Field-oriented mode
            swerve.driveFieldOriented(velocity.getX(), velocity.getY(), omegaRadiansPerSecond);
        } else {
            // Robot-oriented mode
            swerve.driveRobotOriented(velocity.getX(), velocity.getY(), omegaRadiansPerSecond);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        headingSetpointDeg = null;
        rotationSettleTimer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private static Translation2d calculateTeleopVelocity(
            double vX, double vY, double boostFactor, boolean invert) {
        // invert
        if (invert) {
            vX = -vX;
            vY = -vY;
        }

        // handy utilities
        Translation2d input = new Translation2d(vX, vY);
        double magnitude = input.getNorm();
        Rotation2d angle = magnitude > 1e-6 ? input.getAngle() : new Rotation2d();

        // apply boost factor
        magnitude *= boostFactor;

        // handle case where in simulator, a value of 1,1 is possible whereas normally the
        // controller magnitude never exceeds 1
        magnitude = MathUtil.clamp(magnitude, -1, 1);

        // convert from % to mps
        magnitude *= TRANSLATION_CONFIG.maxSpeedMPS();

        // convert to vector
        return new Translation2d(magnitude, angle);
    }
}
