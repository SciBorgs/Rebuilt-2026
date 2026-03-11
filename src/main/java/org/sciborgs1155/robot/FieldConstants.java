package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.alliance;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants {
  // Origin at corner of blue alliance side of field
  public static final Distance LENGTH = Centimeters.of(1755);
  public static final Distance WIDTH = Centimeters.of(805);

  // Origin at corner of red alliance side of field
  public static final double X_ORIGIN = 0;
  public static final double Y_ORIGIN = 0;
  public static final Translation2d ORIGIN = new Translation2d(X_ORIGIN, Y_ORIGIN);

  // Center of hub
  public static final double HUB_X = 461.1624;
  public static final double HUB_Y = 403.4536;
  public static final Translation2d HUB = new Translation2d(HUB_X, HUB_Y);

  // Middle of bottom trench
  public static final double MID_BOTTOM_TRENCH_X = 461.1624;
  public static final double MID_BOTTOM_TRENCH_Y = 63.4238;
  public static final Translation2d MID_BOTTOM_TRENCH =
      new Translation2d(MID_BOTTOM_TRENCH_X, MID_BOTTOM_TRENCH_Y);

  // Middle of upper trench
  public static final double MID_UPPER_TRENCH_X = 461.1624;
  public static final double MID_UPPER_TRENCH_Y = 742.4674;
  public static final Translation2d MID_UPPER_TRENCH =
      new Translation2d(MID_UPPER_TRENCH_X, MID_UPPER_TRENCH_Y);

  // Middle of lower bump
  public static final double MID_LOWER_BUMP_X = 461.1624;
  public static final double MID_LOWER_BUMP_Y = 251.079;
  public static final Translation2d MID_LOWER_BUMP =
      new Translation2d(MID_LOWER_BUMP_X, MID_LOWER_BUMP_Y);

  // Middle of upper bump
  public static final double MID_UPPER_BUMP_X = 461.1624;
  public static final double MID_UPPER_BUMP_Y = 555.1932;
  public static final Translation2d MID_UPPER_BUMP =
      new Translation2d(MID_UPPER_BUMP_X, MID_UPPER_BUMP_Y);

  // Middle of bar between the lower trench and the lower bump
  public static final double MID_BAR1_X = 461.1624;
  public static final double MID_BAR1_Y = 143.129;
  public static final Translation2d MID_BAR1 = new Translation2d(MID_BAR1_X, MID_BAR1_Y);

  // Middle of the bar between the upper trench and the upper bump
  public static final double MID_BAR2_X = 461.1624;
  public static final double MID_BAR2_Y = 662.98572;
  public static final Translation2d MID_BAR2 = new Translation2d(MID_BAR2_X, MID_BAR2_Y);

  // Top edge of hub
  public static final double TOP_HUB_X = 461.1624;
  public static final double TOP_HUB_Y = 477.6343;
  public static final Translation2d TOP_HUB = new Translation2d(TOP_HUB_X, TOP_HUB_Y);

  // Bottom edge of hub
  public static final double BOTTOM_HUB_X = 461.1624;
  public static final double BOTTOM_HUB_Y = 329.2729;
  public static final Translation2d BOTTOM_HUB = new Translation2d(BOTTOM_HUB_X, BOTTOM_HUB_Y);

  // Middle of Alliance Zone fuel station
  public static final double FUEL_STATION_X = 39.37;
  public static final double FUEL_STATION_Y = 594.995;
  public static final Translation2d FUEL_STATION =
      new Translation2d(FUEL_STATION_X, FUEL_STATION_Y);

  // Bottom edge of climb
  public static final double BOTTOM_CLIMB_X = 110.67415;
  public static final double BOTTOM_CLIMB_Y = 314.880625;
  public static final Translation2d BOTTOM_CLIMB =
      new Translation2d(BOTTOM_CLIMB_X, BOTTOM_CLIMB_Y);

  // Top edge of climb
  public static final double TOP_CLIMB_X = 110.67415;
  public static final double TOP_CLIMB_Y = 434.260625;
  public static final Translation2d TOP_CLIMB = new Translation2d(TOP_CLIMB_X, TOP_CLIMB_Y);

  // Middle of climb
  public static final double MID_CLIMB_X = 110.67415;
  public static final double MID_CLIMB_Y = 374.570625;
  public static final Translation2d MID_CLIMB = new Translation2d(MID_CLIMB_X, MID_CLIMB_Y);

  // Middle of outpost
  public static final double MID_OUTPOST_X = 0;
  public static final double MID_OUTPOST_Y = 65.0748;
  public static final Translation2d MID_OUTPOST = new Translation2d(MID_OUTPOST_X, MID_OUTPOST_Y);

  // Prevents instantiation
  private FieldConstants() {}

  /** Returns whether the provided position is within the boundaries of the field. */
  public static boolean inField(Pose3d pose) {
    return pose.getX() > 0
        && pose.getX() < LENGTH.in(Meters)
        && pose.getY() > 0
        && pose.getY() < WIDTH.in(Meters);
  }

  /**
   * Creates a Vector from polar coordinates.
   *
   * @param magnitude The magnitude of the vector.
   * @param direction The direction of the vector.
   * @return A Vector from the given polar coordinates.
   */
  public static Vector<N2> fromPolarCoords(double magnitude, Rotation2d direction) {
    return VecBuilder.fill(magnitude * direction.getCos(), magnitude * direction.getSin());
  }

  /**
   * Rotates a pose 180* with respect to the center of the field, effectively swapping alliances.
   *
   * <p><b> NOTE: This only works for rotated reflect fields like Reefscape, not mirrored fields
   * like Crescendo. </b>
   *
   * @param pose The pose being reflected.
   * @return The reflected pose.
   */
  public static Pose2d allianceReflect(Pose2d pose) {
    return alliance() == Alliance.Blue
        ? pose
        : new Pose2d(
            pose.getTranslation()
                .rotateAround(new Translation2d(LENGTH.div(2), WIDTH.div(2)), Rotation2d.k180deg),
            pose.getRotation().plus(Rotation2d.k180deg));
  }

  /**
   * Reflects width-wise distances through the middle of the field if the alliance is red, otherwise
   * does nothing
   *
   * @param blueDist The input distance, usually for the blue alliance.
   * @return A reflected distance, only if the alliance is red.
   */
  private static Distance reflectDistance(Distance blueDist) {
    return alliance() == Alliance.Blue ? blueDist : WIDTH.minus(blueDist);
  }

  /**
   * Determines the alliance based on the pose's x-coordinate on the field.
   *
   * @param pose The pose to check.
   * @return The alliance corresponding to the pose's position.
   */
  public static Alliance allianceFromPose(Pose2d pose) {
    return pose.getX() > LENGTH.in(Meters) / 2 ? Alliance.Red : Alliance.Blue;
  }

  /**
   * A transform that will translate the pose robot-relative right by a certain distance. Negative
   * distances will move the pose left.
   *
   * @distance The distance that the pose will be moved.
   * @return A transform to strafe a pose.
   */
  public static Transform2d strafe(Distance distance) {
    return new Transform2d(
        new Translation2d(distance.in(Meters), Rotation2d.kCW_90deg), Rotation2d.kZero);
  }

  /**
   * A transform that will translate the pose robot-relative forward by a certain distance. Negative
   * distances will move the pose backward.
   *
   * @distance The distance that the pose will be moved.
   * @return A transform to move a pose forward.
   */
  public static Transform2d advance(Distance distance) {
    return new Transform2d(
        new Translation2d(distance.in(Meters), Rotation2d.kZero), Rotation2d.kZero);
  }

  // List field constants below!
}
