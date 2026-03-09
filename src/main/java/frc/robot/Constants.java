// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_X;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_Y;

import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.core.config.EncoderConfig;
import ca.team1310.swerve.core.config.ModuleConfig;
import ca.team1310.swerve.core.config.MotorConfig;
import ca.team1310.swerve.core.config.MotorType;
import ca.team1310.swerve.core.config.TelemetryLevel;
import ca.team1310.swerve.gyro.config.GyroConfig;
import ca.team1310.swerve.utils.Coordinates;
import ca.team1310.swerve.vision.config.LimelightConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveDriveSubsystemConfig;
import frc.robot.subsystems.swerve.SwerveRotationConfig;
import frc.robot.subsystems.swerve.SwerveTranslationConfig;
import frc.robot.subsystems.vision.VisionConfig;
import frc.robot.subsystems.vision.VisionTelemetryLevel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class TelemetryConfig {
    public static boolean drive = true;
    public static VisionTelemetryLevel vision = VisionTelemetryLevel.NONE;
    public static TelemetryLevel swerve = TelemetryLevel.VERBOSE;
    public static boolean test = false;
    public static boolean oi = false;
    public static boolean coral = false;
    public static boolean climb = false;
    public static boolean pneumatics = false;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = .15;

    public static final double GENERAL_SPEED_FACTOR = .5;
    public static final double MAX_SPEED_FACTOR = 1;
    public static final double SLOW_SPEED_FACTOR = .1;
  }

  public static final class LightingConstants {

    public static final int LED_STRING_PWM_PORT = 9;
    public static final int LED_STRING_LENGTH = 36;
  }

  public static final class FieldConstants {
    public static final double FIELD_EXTENT_METRES_Y = 8.07;
    public static final double FIELD_EXTENT_METRES_X = 16.54;
  }

  public static final class AutoConstants {

    public enum AutoPattern {
      DO_NOTHING,
      EXIT_ZONE,
      SIMPLE_CENTER,
      OPPORTUNISTIC_OUTPOST,
      SHOOT_CENTER,
      LEFT_SHOOT_CLIMB,
      RIGHT_SHOOT_CLIMB,
      CUSTOM
    }

    public enum Delay {
      NO_DELAY,
      WAIT_0_5_SECOND,
      WAIT_1_SECOND,
      WAIT_1_5_SECONDS,
      WAIT_2_SECONDS,
      WAIT_2_5_SECONDS,
      WAIT_3_SECONDS,
      WAIT_5_SECONDS
    }
  }

  public static final class ShooterConstants {

    public static final int SHOOTER_PRIMARY_MOTOR_CAN_ID = 30;
    public static final int SHOOTER_SECONDARY_MOTOR_CAN_ID = 31;
    public static final int KICKER_MOTOR_PWM_PORT = 0;
    public static final int HOOD_PWM_PORT = 4;
    public static final int AGITATOR_PWM_PORT = 5;

    public static final double MAX_SHOOTER_RPM = 6300;
    public static final double KP = 0.00015;
    public static final double KI = 0.00001;
    public static final double KD = 0; // keep this at zero
    public static final double KFF = 1 / MAX_SHOOTER_RPM;
    public static final double KICKER_RUNSPEED = 1;
    public static final double AGITATOR_RUNSPEED = -0.6 * 0; // FIXME: remove 0 later
    public static final double ACCPETED_SHOOTER_ERROR = 50.0;

    // Control values
    // public static final double SLOPE_VALUE_SUPER_FAR = 479.67;
    // public static final double Y_INT_SUPER_FAR = 2867.33;
    // public static final double SLOPE_VALUE_MID = 570.14;
    // public static final double Y_INT_MID = 2816.44;
    // public static final double SLOPE_VALUE_CLOSE = 733.5;
    // public static final double Y_INT_CLOSE = 2882.2;

    // Test 1 values (lower y int)
    // public static final double SLOPE_VALUE_SUPER_FAR = 479.67;
    // public static final double Y_INT_SUPER_FAR = 2767.33;
    // public static final double SLOPE_VALUE_MID = 570.14;
    // public static final double Y_INT_MID = 2716.44;
    // public static final double SLOPE_VALUE_CLOSE = 733.5;
    // public static final double Y_INT_CLOSE = 2782.2;

    // Test 2 values (lower slope)
    // public static final double SLOPE_VALUE_SUPER_FAR = 419.67;
    // public static final double Y_INT_SUPER_FAR = 2867.33;
    // public static final double SLOPE_VALUE_MID = 510.14;
    // public static final double Y_INT_MID = 2816.44;
    // public static final double SLOPE_VALUE_CLOSE = 670.5;
    // public static final double Y_INT_CLOSE = 2882.2;

    // Test 3 values (higher int)
    // public static final double SLOPE_VALUE_SUPER_FAR = 479.67;
    // public static final double Y_INT_SUPER_FAR = 2967.33;
    // public static final double SLOPE_VALUE_MID = 570.14;
    // public static final double Y_INT_MID = 2916.44;
    // public static final double SLOPE_VALUE_CLOSE = 733.5;
    // public static final double Y_INT_CLOSE = 2982.2;

    // Test 4 values (higher slope)
    // public static final double SLOPE_VALUE_SUPER_FAR = 520.67;
    // public static final double Y_INT_SUPER_FAR = 2867.33;
    // public static final double SLOPE_VALUE_MID = 620.14;
    // public static final double Y_INT_MID = 2816.44;
    // public static final double SLOPE_VALUE_CLOSE = 780.5;
    // public static final double Y_INT_CLOSE = 2882.2;

    // Test 5 values (higher slope lower y int)
    // public static final double SLOPE_VALUE_SUPER_FAR = 520.67;
    // public static final double Y_INT_SUPER_FAR = 2767.33;
    // public static final double SLOPE_VALUE_MID = 620.14;
    // public static final double Y_INT_MID = 2716.44;
    // public static final double SLOPE_VALUE_CLOSE = 780.5;
    // public static final double Y_INT_CLOSE = 2782.2;

    // Test values 6 (hiegher y int, lower slope)
    // public static final double SLOPE_VALUE_SUPER_FAR = 400.67;
    // public static final double Y_INT_SUPER_FAR = 2967.33;
    // public static final double SLOPE_VALUE_MID = 500.14;
    // public static final double Y_INT_MID = 2916.44;
    // public static final double SLOPE_VALUE_CLOSE = 670.5;
    // public static final double Y_INT_CLOSE = 2982.2;

    public static final double SLOPE_VALUE_SUPER_FAR = 479.67; // CHANGE ALL SLOPES AND Y INTS *****
    public static final double Y_INT_SUPER_FAR = 2867.33;
    public static final double SUPER_FAR_SHOOTING_DISTANCE = 3.5;
    public static final double SUPER_FAR_SHOOT_HOOD_VALUE = 0.6;

    public static final double SLOPE_VALUE_MID = 570.14;
    public static final double Y_INT_MID = 2816.44;
    public static final double MEDIUM_SHOOTING_DISTANCE = 2.0;
    public static final double MEDIUM_SHOOT_HOOD_VALUE = 0.35;

    public static final double SLOPE_VALUE_CLOSE = 733.5;
    public static final double Y_INT_CLOSE = 2882.2;
    public static final double CLOSE_SHOOT_HOOD_VALUE = 0.0;

    public static final double MAX_SHOOTING_DISTANCE = 10.0;
  }

  public static final class Swerve {

    /** Front to back from the middle of the wheels */
    public static final double WHEEL_BASE_METRES = inchesToMeters(22.75);

    /** Side to side from the middle of the wheels */
    public static final double TRACK_WIDTH_METRES = inchesToMeters(18.75);

    public static final double SDS_MK4I_WHEEL_RADIUS_M = 0.051;

    public static final GyroConfig GYRO_CONFIG = GyroConfig.pigeon2(8, true);

    public static final SwerveTranslationConfig TRANSLATION_CONFIG = new SwerveTranslationConfig(
        /* tolerance (m) */ 0.02,
        /* min speed (m/s) */ 1.0,
        /* max speed (m/s) */ 4.8,
        /* max module speed (m/s) */ 5.36,
        /* max acceleration (m/s/s) */ 10.0,
        /* velocity PID p */ 1.2,
        /* velocity PID i */ 0,
        /* velocity PID d */ 0);

    public static final SwerveRotationConfig ROTATION_CONFIG = new SwerveRotationConfig(
        /* max rot vel (rad/s) */ Rotation2d.fromRotations(1.5).getRadians(),
        /* defaultRotVelocityRadPS (rad/s) */ Rotation2d.fromRotations(0.75).getRadians(),
        /* max rotation accel (rad/s/s) */ Rotation2d.fromRotations(2).getRadians(),
        /* heading PID p */ 0.033, // Rads/Deg
        /* heading PID i */ 0,
        /* heading PID d */ 0);

    private static final MotorConfig ANGLE_MOTOR_CONFIG = new MotorConfig(
        /* motor hardware type */ MotorType.NEO_SPARK_MAX,
        /* inverted? */ true,
        /* current limit (A) */ 20,
        /* nominal voltage (V) */ 12,
        /* ramp rate 0 to full power (s) */ 0.02,
        /* angle motor gear ratio */ 150.0 / 7 /* SDS MK4i 150/7:1 */,
        /* angle motor PID p */ 0.009,
        /* angle motor PID i */ 0,
        /* angle motor PID d */ 0,
        /* angle motor PID ff */ 0,
        /* angle motor PID izone */ 0);

    private static final MotorConfig DRIVE_MOTOR_CONFIG = new MotorConfig(
        /* motor hardware type */ MotorType.NEO_SPARK_FLEX,
        /* inverted? */ false,
        /* current limit (A) */ 40,
        /* nominal voltage (V) */ 12,
        /* ramp rate 0 to full power (s) */ 0.01,
        /* drive motor gear ratio */ 6.75 /* SDS MK4i L2 --> 6.75:1 */,
        /* drive motor PID p */ 0.075,
        /* drive motor PID i */ 0,
        /* drive motor PID d */ 0,
        /* drive motor PID ff */ 1 / TRANSLATION_CONFIG.maxModuleSpeedMPS(),
        /* drive motor PID izone */ 0);

    private static final EncoderConfig ANGLE_ENCODER_CONFIG = new EncoderConfig(false, 0.005, 5);

    public static final ModuleConfig FRONT_LEFT = new ModuleConfig(
        "frontleft",
        new Coordinates(TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2),
        SDS_MK4I_WHEEL_RADIUS_M,
        10,
        DRIVE_MOTOR_CONFIG,
        11,
        ANGLE_MOTOR_CONFIG,
        12,
        Rotation2d.fromRotations(0.724121).getDegrees(),
        ANGLE_ENCODER_CONFIG);

    public static final ModuleConfig FRONT_RIGHT = new ModuleConfig(
        "frontright",
        new Coordinates(TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2),
        SDS_MK4I_WHEEL_RADIUS_M,
        15,
        DRIVE_MOTOR_CONFIG,
        16,
        ANGLE_MOTOR_CONFIG,
        17,
        Rotation2d.fromRotations(0.115967).getDegrees(),
        ANGLE_ENCODER_CONFIG);

    public static final ModuleConfig BACK_RIGHT = new ModuleConfig(
        "backright",
        new Coordinates(-TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2),
        SDS_MK4I_WHEEL_RADIUS_M,
        20,
        DRIVE_MOTOR_CONFIG,
        21,
        ANGLE_MOTOR_CONFIG,
        22,
        Rotation2d.fromRotations(0.583496).getDegrees(),
        ANGLE_ENCODER_CONFIG);

    public static final ModuleConfig BACK_LEFT = new ModuleConfig(
        "backleft",
        new Coordinates(-TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2),
        SDS_MK4I_WHEEL_RADIUS_M,
        25,
        DRIVE_MOTOR_CONFIG,
        26,
        ANGLE_MOTOR_CONFIG,
        27,
        Rotation2d.fromRotations(0.632812).getDegrees(),
        ANGLE_ENCODER_CONFIG);

    public static final CoreSwerveConfig CORE_SWERVE_CONFIG = new CoreSwerveConfig(
        WHEEL_BASE_METRES,
        TRACK_WIDTH_METRES,
        SDS_MK4I_WHEEL_RADIUS_M,
        Robot.kDefaultPeriod,
        TRANSLATION_CONFIG.maxModuleSpeedMPS(),
        TRANSLATION_CONFIG.maxSpeedMPS(),
        ROTATION_CONFIG.maxRotVelocityRadPS(),
        0.55,
        0.85,
        0.65,
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        TelemetryConfig.swerve);

    private static final LimelightConfig LIMELIGHT_CONFIG = new LimelightConfig(
        VisionConstants.VISION_PRIMARY_LIMELIGHT_NAME,
        FIELD_EXTENT_METRES_X,
        FIELD_EXTENT_METRES_Y);

    public static final SwerveDriveSubsystemConfig SUBSYSTEM_CONFIG = new SwerveDriveSubsystemConfig(
        true,
        CORE_SWERVE_CONFIG,
        GYRO_CONFIG,
        LIMELIGHT_CONFIG,
        TRANSLATION_CONFIG,
        ROTATION_CONFIG,
        TelemetryConfig.drive);
  }

  public static final class VisionConstants {
    public static final VisionConfig VISION_CONFIG = new VisionConfig(0, 0, 0.7, 0.1, .5, true,
        Constants.TelemetryConfig.vision);

    // TODO: fixme: rename me
    public static final String VISION_PRIMARY_LIMELIGHT_NAME = "hopper";
    public static final String VISION_SECONDARY_LIMELIGHT_NAME = "hugh";
  }

  public static final class IntakeConstants {

    public static final int DOOR_CAN_ID = 41;
    public static final int TOP_ROLLER_PWM_PORT = 3;
    public static final int BOTTOM_ROLLER_PWM_PORT = 1;
    public static final int DOOR_CLOSED_LIMIT_DIO_PORT = 0;

    public static final int INTAKE_SPEED = -1;
    public static final double INTAKE_DOOR_ANGLE = 40.0;
    public static final double DOOR_KP = 0.01; // Making this big so it go weeeee, the last kp was 0.01
    public static final double DOOR_KI = 0;
    public static final double DOOR_KD = 0;
    public static final double DOOR_ENCODERS_TO_DEGREES = 360 / 90;
  }
}
