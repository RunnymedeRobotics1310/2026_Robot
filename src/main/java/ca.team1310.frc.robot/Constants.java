// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.team1310.frc.robot;

import ca.team1310.frc.robot.telemetry.VisionTelemetryLevel;
import ca.team1310.swerve.core.config.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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

  public static final class RobotConfig {

    public static final double LENGTH_METRES = 0.71;
    public static final double WIDTH_METRES = 0.71;
    public static final double HEIGHT_METRES = 0.4;
    public static final double BUMPER_WIDTH = 0.0;
  }

  public static final class VisionConstants {

    //        public static final VisionConfig VISION_CONFIG = new VisionConfig(
    //            0,
    //            0,
    //            0.7,
    //            0.1,
    //            .5,
    //            true,
    //            Constants.TelemetryConfig.vision
    //        );
    //
    public static final String VISION_PRIMARY_LIMELIGHT_NAME = "nikola";
    public static final String VISION_SECONDARY_LIMELIGHT_NAME = "thomas";
  }

  public static final class OiConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double CONTROLLER_DEADBAND = .2;

    /**
     * Standard drive speed factor. Regular teleop drive will use this factor of the max
     * translational speed.
     */
    public static final double GENERAL_SPEED_FACTOR = .5;

    /**
     * Maximum drive speed factor. When boosting, this factor will be multiplied against the max
     * translational speed.
     */
    public static final double MAX_SPEED_FACTOR = 1;

    /**
     * Slow mode drive speed factor. When running in slow mode, this factor will be multiplied
     * against the max translational speed.
     */
    public static final double SLOW_SPEED_FACTOR = .1;
  }

  public static final class FieldConstants {

    public static final double FIELD_EXTENT_METRES_Y = 8.07;
    public static final double FIELD_EXTENT_METRES_X = 16.54;

    // This is physical tag locations on field, from 2025FieldDrawings.pdf but the heading is
    // swapped 180 degrees to indicate heading to face the tag, vs the orientation the tag is facing
    //TODO: fixme: update these to 2026 tags
    public enum TAGS {
      RED_LEFT_SOURCE(1, new Pose2d(16.697198, 0.655320, Rotation2d.fromDegrees(-54))),
      RED_RIGHT_SOURCE(2, new Pose2d(16.697198, 7.396480, Rotation2d.fromDegrees(54))),
      RED_PROCESSOR(3, new Pose2d(11.560810, 8.055610, Rotation2d.fromDegrees(90))),
      RED_RIGHT_BARGE(4, new Pose2d(9.276080, 6.137656, Rotation2d.fromDegrees(180))),
      RED_LEFT_BARGE(5, new Pose2d(9.276080, 1.914906, Rotation2d.fromDegrees(180))),
      RED_LEFT_REEF_2_3(6, new Pose2d(13.474446, 3.306318, Rotation2d.fromDegrees(120))),
      RED_LEFT_RIGHT_REEF_1(7, new Pose2d(13.890498, 4.025900, Rotation2d.fromDegrees(180))),
      RED_RIGHT_REEF_2_3(8, new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(-120))),
      RED_RIGHT_REEF_4_5(9, new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(-60))),
      RED_RIGHT_LEFT_REEF_6(10, new Pose2d(12.227306, 4.025900, Rotation2d.fromDegrees(0))),
      RED_LEFT_REEF_4_5(11, new Pose2d(12.643358, 3.306318, Rotation2d.fromDegrees(60))),
      BLUE_RIGHT_SOURCE(12, new Pose2d(0.851154, 0.655320, Rotation2d.fromDegrees(-126))),
      BLUE_LEFT_SOURCE(13, new Pose2d(0.851154, 7.396480, Rotation2d.fromDegrees(126))),
      BLUE_LEFT_BARGE(14, new Pose2d(8.272272, 6.137656, Rotation2d.fromDegrees(0))),
      BLUE_RIGHT_BARGE(15, new Pose2d(8.272272, 1.914906, Rotation2d.fromDegrees(0))),
      BLUE_PROCESSOR(16, new Pose2d(5.987542, -0.003810, Rotation2d.fromDegrees(-90))),
      BLUE_RIGHT_REEF_2_3(17, new Pose2d(4.073906, 3.306318, Rotation2d.fromDegrees(60))),
      BLUE_RIGHT_LEFT_REEF_1(18, new Pose2d(3.657600, 4.025900, Rotation2d.fromDegrees(0))),
      BLUE_LEFT_REEF_2_3(19, new Pose2d(4.073906, 4.745482, Rotation2d.fromDegrees(-60))),
      BLUE_LEFT_REEF_4_5(20, new Pose2d(4.904740, 4.745482, Rotation2d.fromDegrees(-120))),
      BLUE_LEFT_RIGHT_REEF_6(21, new Pose2d(5.321046, 4.025900, Rotation2d.fromDegrees(180))),
      BLUE_RIGHT_REEF_4_5(22, new Pose2d(4.904740, 3.306318, Rotation2d.fromDegrees(120)));

      private static final Map<Integer, TAGS> lookup = new HashMap<>();

      static {
        for (TAGS tag : TAGS.values()) {
          lookup.put(tag.tagId, tag);
        }
      }

      public final int tagId;
      public final Pose2d pose;

      TAGS(int tagId, Pose2d pose) {
        this.tagId = tagId;
        this.pose = pose;
      }

      public static TAGS getTagById(int tagId) {
        return lookup.get(tagId);
      }

      public static boolean isValidTagId(int tagId) {
        return lookup.containsKey(tagId);
      }
    }
  }

  public static final class Swerve {
  }

  public static final class AutoConstants {

    public enum AutoPattern {
      DO_NOTHING,
      EXIT_ZONE,
    }

    public enum Delay {
      NO_DELAY,
      WAIT_0_5_SECOND,
      WAIT_1_SECOND,
      WAIT_1_5_SECONDS,
      WAIT_2_SECONDS,
      WAIT_2_5_SECONDS,
      WAIT_3_SECONDS,
      WAIT_5_SECONDS,
    }

    public enum FieldLocation {
      // Generalized Multi Alliance Locations
      EXAMPLE_POSE(new Pose2d(1, 2, Rotation2d.k180deg));

      public final Pose2d pose;
      public final int blueTagId;
      public final int redTagId;
      public final boolean isLeftSide;

      FieldLocation(Pose2d pose) {
        this(pose, 0, 0, false);
      }

      FieldLocation(Pose2d pose, int blueTagId, int redTagId) {
        this(pose, blueTagId, redTagId, false);
      }

      FieldLocation(Pose2d pose, int blueTagId, int redTagId, boolean isLeftSide) {
        this.pose = pose;
        this.blueTagId = blueTagId;
        this.redTagId = redTagId;
        this.isLeftSide = isLeftSide;
      }
    }
  }

  public enum BotTarget {

    EXAMPLE_TARGET(new Translation3d(8.16, 7.47, 0)),

    // When No Target is Set
    NONE(new Translation3d(0, 0, 0)),

    // No focus, but go to any tag visible
    ALL(new Translation3d(0, 0, 0));

    private final Translation3d location;

    BotTarget(Translation3d location) {
      this.location = location;
    }

    public Translation3d getLocation() {
      return location;
    }

    @Override
    public String toString() {
      return "BotTarget: " + name() + " at " + location;
    }
  }
}
