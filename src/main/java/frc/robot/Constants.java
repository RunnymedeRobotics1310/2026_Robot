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
                public static final double CONTROLLER_DEADBAND = .2;

                public static final double GENERAL_SPEED_FACTOR = .5;
                public static final double MAX_SPEED_FACTOR = 1;
                public static final double SLOW_SPEED_FACTOR = .1;
        }

        public static final class LightingConstants {

                public static final int LED_STRING_PWM_PORT = 9;
                public static final int LED_STRING_LENGTH = 38;

                public static final int LED_CLIMB_VIEW_START = 0;
                public static final int LED_CLIMB_VIEW_END = 4;

                public static final int LED_INTAKE_VIEW_START = 6;
                public static final int LED_INTAKE_VIEW_END = 10;

                public static final int LED_ROBOT_HEALTH_VIEW_START = 12;
                public static final int LED_ROBOT_HEALTH_VIEW_END = 16;

                public static final int LED_DRIVE_VIEW_START = 18;
                public static final int LED_DRIVE_VIEW_END = 20;
        }

        public static final class FieldConstants {
                public static final double FIELD_EXTENT_METRES_Y = 8.07;
                public static final double FIELD_EXTENT_METRES_X = 16.54;
        }

        public static final class ShooterConstants {
                public static final int maxShooterSpeedRpm = 5700;
                public static final float Kp = 0.5f; // proportional gain constant for pid controller
                public static final float calcSlope = 240.0f; // slope for shooting speed calculation
                public static final float calcYIntercept = 300; // y-intercept for shooting speed calculation
        }

        public static final class Swerve {

                /** Front to back from the middle of the wheels */
                public static final double WHEEL_BASE_METRES = inchesToMeters(16.75);

                /** Side to side from the middle of the wheels */
                public static final double TRACK_WIDTH_METRES = inchesToMeters(16.75);

                public static final double SDS_MK4I_WHEEL_RADIUS_M = 0.0485;

                public static final GyroConfig GYRO_CONFIG = GyroConfig.pigeon2(8, true);

                public static final SwerveTranslationConfig TRANSLATION_CONFIG = new SwerveTranslationConfig(
                                /* tolerance (m) */ 0.02,
                                /* min speed (m/s) */ 1.0,
                                /* max speed (m/s) */ 4.8,
                                /* max module speed (m/s) */ 5.36,
                                /* max acceleration (m/s/s) */ 42.0,
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
                                /* ramp rate 0 to full power (s) */ 0.25,
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
                                /* ramp rate 0 to full power (s) */ 0.25, // TODO: FIXME: TRY LOWERING THIS A LOT
                                /* drive motor gear ratio */ 6.75 /* SDS MK4i L2 --> 6.75:1 */,
                                /* drive motor PID p */ 0.075,
                                /* drive motor PID i */ 0,
                                /* drive motor PID d */ 0,
                                /* drive motor PID ff */ 1 / TRANSLATION_CONFIG.maxModuleSpeedMPS(),
                                /* drive motor PID izone */ 0);

                private static final EncoderConfig ANGLE_ENCODER_CONFIG = new EncoderConfig(false, 0.005, 5);

                public static final ModuleConfig FRONT_LEFT = new ModuleConfig(
                                "frontleft",
                                new Coordinates(-TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2),
                                SDS_MK4I_WHEEL_RADIUS_M,
                                10,
                                DRIVE_MOTOR_CONFIG,
                                11,
                                ANGLE_MOTOR_CONFIG,
                                12,
                                Rotation2d.fromRotations(0.281982).getDegrees(),
                                ANGLE_ENCODER_CONFIG);

                public static final ModuleConfig FRONT_RIGHT = new ModuleConfig(
                                "frontright",
                                new Coordinates(-TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2),
                                SDS_MK4I_WHEEL_RADIUS_M,
                                15,
                                DRIVE_MOTOR_CONFIG,
                                16,
                                ANGLE_MOTOR_CONFIG,
                                17,
                                Rotation2d.fromRotations(0.411377).getDegrees(),
                                ANGLE_ENCODER_CONFIG);

                public static final ModuleConfig BACK_RIGHT = new ModuleConfig(
                                "backright",
                                new Coordinates(TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2),
                                SDS_MK4I_WHEEL_RADIUS_M,
                                20,
                                DRIVE_MOTOR_CONFIG,
                                21,
                                ANGLE_MOTOR_CONFIG,
                                22,
                                Rotation2d.fromRotations(0.353271).getDegrees(),
                                ANGLE_ENCODER_CONFIG);

                public static final ModuleConfig BACK_LEFT = new ModuleConfig(
                                "backleft",
                                new Coordinates(TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2),
                                SDS_MK4I_WHEEL_RADIUS_M,
                                25,
                                DRIVE_MOTOR_CONFIG,
                                26,
                                ANGLE_MOTOR_CONFIG,
                                27,
                                Rotation2d.fromRotations(0.506836).getDegrees(),
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
                                0.5,
                                0.65,
                                FRONT_LEFT,
                                FRONT_RIGHT,
                                BACK_LEFT,
                                BACK_RIGHT,
                                TelemetryConfig.swerve);

                private static final LimelightConfig LIMELIGHT_CONFIG = new LimelightConfig(
                                VisionConstants.VISION_PRIMARY_LIMELIGHT_NAME, FIELD_EXTENT_METRES_X,
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

        }
}
