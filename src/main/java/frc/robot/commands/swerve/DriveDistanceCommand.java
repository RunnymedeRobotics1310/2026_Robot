package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.auto.config.AutoConfigurable;
import frc.robot.commands.auto.config.ConfigParam;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@AutoConfigurable(value = "drive_distance", category = "drive",
    description = "Drive a fixed distance using odometry")
public class DriveDistanceCommand extends LoggingCommand {

    private static final double CONTROL_DELAY_SECONDS = 0.06;
    private static final double STOP_MARGIN_METRES = 0.02;
    private static final double EFFECTIVE_DECEL_SCALE = 1.0;
    private static final double MIN_EFFECTIVE_DECEL_MPS2 = 0.25;
    private static final double SPEED_FILTER_ALPHA = 0.35;

    private final SwerveSubsystem swerve;
    private final double direction;
    private final double speedMPS;
    private final double distanceMetres;
    private final double heading;
    private final double timeoutSeconds;

    private double allianceDirection;
    private double allianceHeading;
    private Pose2d startPose;
    private double filteredMeasuredSpeedMPS;

    public DriveDistanceCommand(
            SwerveSubsystem swerve,
            @ConfigParam(value = "direction", unit = "deg",
                description = "Field-oriented direction of travel") double direction,
            @ConfigParam(value = "speedMPS", unit = "m/s", min = 0, max = 5.36,
                description = "Translation speed") double speedMPS,
            @ConfigParam(value = "distanceMetres", unit = "m", min = 0, max = 20,
                description = "Distance to travel") double distanceMetres,
            @ConfigParam(value = "headingDegrees", unit = "deg",
                description = "Robot heading to hold") double heading,
            @ConfigParam(value = "timeoutSeconds", unit = "s", min = 0, max = 15,
                description = "Safety timeout") double timeoutSeconds) {
        this.swerve = swerve;
        this.direction = direction;
        this.speedMPS = speedMPS;
        this.distanceMetres = distanceMetres;
        this.heading = heading;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        double headingOffset = 0;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            headingOffset = 180;
        }
        allianceDirection = direction + headingOffset;
        allianceHeading = heading + headingOffset;
        startPose = swerve.getPose();
        filteredMeasuredSpeedMPS = 0;

        logCommandStart("dir=" + allianceDirection + " spd=" + speedMPS
                + " dist=" + distanceMetres + " hdg=" + allianceHeading);
    }

    @Override
    public void execute() {
        double dirRad = Math.toRadians(allianceDirection);
        double vx = speedMPS * Math.cos(dirRad);
        double vy = speedMPS * Math.sin(dirRad);
        double omega = swerve.computeOmega(allianceHeading);
        swerve.driveFieldOriented(vx, vy, omega);
    }

    @Override
    public boolean isFinished() {
        double dx = swerve.getPose().getX() - startPose.getX();
        double dy = swerve.getPose().getY() - startPose.getY();
        double traveled = Math.sqrt(dx * dx + dy * dy);
        double remaining = distanceMetres - traveled;

        double measuredSpeed = Math.max(0, swerve.getMeasuredTranslationSpeedMPS());
        filteredMeasuredSpeedMPS = SPEED_FILTER_ALPHA * measuredSpeed
                + (1 - SPEED_FILTER_ALPHA) * filteredMeasuredSpeedMPS;
        double effectiveDecelMPS2 = Math.max(
                MIN_EFFECTIVE_DECEL_MPS2,
                Constants.Swerve.TRANSLATION_CONFIG.maxAccelMPS2() * EFFECTIVE_DECEL_SCALE);
        double stopDistance = calculateStopDistanceMetres(
                filteredMeasuredSpeedMPS,
                effectiveDecelMPS2,
                CONTROL_DELAY_SECONDS,
                STOP_MARGIN_METRES);

        if (remaining <= stopDistance) {
            setFinishReason("Distance reached: " + format(traveled) + "m"
                    + " remaining=" + format(remaining)
                    + " stopDist=" + format(stopDistance));
            return true;
        }
        if (timeoutSeconds > 0 && hasElapsed(timeoutSeconds)) {
            setFinishReason("Timeout after " + timeoutSeconds + "s");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        logCommandEnd(interrupted);
    }

    public static double calculateStopDistanceMetres(
            double speedMPS,
            double effectiveDecelMPS2,
            double controlDelaySeconds,
            double stopMarginMetres) {
        double speed = Math.max(0, speedMPS);
        double decel = Math.max(MIN_EFFECTIVE_DECEL_MPS2, effectiveDecelMPS2);
        double delay = Math.max(0, controlDelaySeconds);
        double margin = Math.max(0, stopMarginMetres);
        double brakingDistance = (speed * speed) / (2 * decel);
        return brakingDistance + speed * delay + margin;
    }
}
