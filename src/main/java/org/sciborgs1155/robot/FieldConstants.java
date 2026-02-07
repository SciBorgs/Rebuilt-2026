package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.alliance;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// 6328 is a goated team
public final class FieldConstants {
  // Origin at corner of blue alliance side of field
  public static final Distance LENGTH = Centimeters.of(1755);
  public static final Distance WIDTH = Centimeters.of(805);

  // Prevents instantiation
  private FieldConstants() {}

  public static final String FIELD_TYPE = "welded";

  // AprilTag related constants
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final int TAG_COUNT = FIELD_LAYOUT.getTags().size();
  public static final double TAG_WIDTH = Units.inchesToMeters(6.5);

  // Field dimensions
  public static final double FIELD_LENGTH = FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH = FIELD_LAYOUT.getFieldWidth();

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double CENTER = FIELD_LENGTH / 2.0;
    public static final double STARTING = FIELD_LAYOUT.getTagPose(26).get().getX();
    public static final double ALLIANCE_ZONE = STARTING;
    public static final double HUB_CENTER =
        FIELD_LAYOUT.getTagPose(26).get().getX() + Hub.WIDTH / 2.0;
    public static final double NEUTRAL_ZONE_NEAR = CENTER - Units.inchesToMeters(120);
    public static final double NEUTRAL_ZONE_FAR = CENTER + Units.inchesToMeters(120);
    public static final double OPP_HUB_CENTER =
        FIELD_LAYOUT.getTagPose(4).get().getX() + Hub.WIDTH / 2.0;
    public static final double OPP_ALLIANCE_ZONE = FIELD_LAYOUT.getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double CENTER = FIELD_WIDTH / 2.0;

    // Right of hub
    public static final double RIGHT_BUMP_START = Hub.NEAR_RIGHT_CORNER.getY();
    public static final double RIGHT_BUMP_END = RIGHT_BUMP_START - RightBump.WIDTH;
    public static final double RIGHT_TRENCH_OPEN_START =
        RIGHT_BUMP_END - Units.inchesToMeters(12.0);
    public static final double RIGHT_TRENCH_OPEN_END = 0;

    // Left of hub
    public static final double LEFT_BUMP_END = Hub.NEAR_LEFT_CORNER.getY();
    public static final double LEFT_BUMP_START = LEFT_BUMP_END + LeftBump.WIDTH;
    public static final double LEFT_TRENCH_OPEN_END = LEFT_BUMP_START + Units.inchesToMeters(12.0);
    public static final double LEFT_TRENCH_OPEN_START = FIELD_WIDTH;
  }

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double INNER_WIDTH = Units.inchesToMeters(41.7);
    public static final double INNER_HEIGHT = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d TOP_CENTER_POINT =
        new Translation3d(
            FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0, HEIGHT);
    public static final Translation3d INNER_CENTER_POINT =
        new Translation3d(
            FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            INNER_HEIGHT);

    public static final Translation2d NEAR_LEFT_CORNER =
        new Translation2d(TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d NEAR_RIGHT_CORNER =
        new Translation2d(TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);
    public static final Translation2d FAR_LEFT_CORNER =
        new Translation2d(TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d FAR_RIGHT_CORNER =
        new Translation2d(TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d OPP_TOP_CENTER_POINT =
        new Translation3d(
            FIELD_LAYOUT.getTagPose(4).get().getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0, HEIGHT);
    public static final Translation2d OPP_NEAR_LEFT_CORNER =
        new Translation2d(
            OPP_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d OPP_NEAR_RIGHT_CORNER =
        new Translation2d(
            OPP_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);
    public static final Translation2d OPP_FAR_LEFT_CORNER =
        new Translation2d(
            OPP_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d OPP_FAR_RIGHT_CORNER =
        new Translation2d(
            OPP_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);

    // Hub faces
    public static final Pose2d NEAR_FACE = FIELD_LAYOUT.getTagPose(26).get().toPose2d();
    public static final Pose2d FAR_FACE = FIELD_LAYOUT.getTagPose(20).get().toPose2d();
    public static final Pose2d RIGHT_FACE = FIELD_LAYOUT.getTagPose(18).get().toPose2d();
    public static final Pose2d LEFT_FACE = FIELD_LAYOUT.getTagPose(21).get().toPose2d();
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(73.0);
    public static final double HEIGHT = Units.inchesToMeters(6.513);
    public static final double DEPTH = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d NEAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER - WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER;
    public static final Translation2d FAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER + WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER;

    // Relevant reference points on opposing side
    public static final Translation2d OPP_NEAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER - WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d OPP_NEAR_RIGHT_CORNER = Hub.OPP_NEAR_LEFT_CORNER;
    public static final Translation2d OPP_FAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER + WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d OPP_FAR_RIGHT_CORNER = Hub.OPP_FAR_LEFT_CORNER;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(73.0);
    public static final double HEIGHT = Units.inchesToMeters(6.513);
    public static final double DEPTH = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d NEAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER + WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER;
    public static final Translation2d FAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER - WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER;

    // Relevant reference points on opposing side
    public static final Translation2d OPP_NEAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER + WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d OPP_NEAR_RIGHT_CORNER = Hub.OPP_NEAR_LEFT_CORNER;
    public static final Translation2d OPP_FAR_LEFT_CORNER =
        new Translation2d(LinesVertical.HUB_CENTER - WIDTH / 2, Units.inchesToMeters(255));
    public static final Translation2d OPP_FAR_RIGHT_CORNER = Hub.OPP_FAR_LEFT_CORNER;
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(65.65);
    public static final double DEPTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT = Units.inchesToMeters(40.25);
    public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
    public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
    public static final Translation3d OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT);

    // Relevant reference points on opposing side
    public static final Translation3d OPP_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.OPP_HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
    public static final Translation3d OPP_OPENING_TOP_RIGHT =
        new Translation3d(
            LinesVertical.OPP_HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT);
  }

  public static class RightTrench {

    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(65.65);
    public static final double DEPTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT = Units.inchesToMeters(40.25);
    public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
    public static final double OPENING_HEIGH = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGH);
    public static final Translation3d OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, 0, OPENING_HEIGH);

    // Relevant reference points on opposing side
    public static final Translation3d OPP_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.OPP_HUB_CENTER, OPENING_WIDTH, OPENING_HEIGH);
    public static final Translation3d OPP_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.OPP_HUB_CENTER, 0, OPENING_HEIGH);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(49.25);
    public static final double DEPTH = Units.inchesToMeters(45.0);
    public static final double HEIGHT = Units.inchesToMeters(78.25);
    public static final double INNER_OPENING_WIDTH = Units.inchesToMeters(32.250);
    public static final double FRONT_FACE_X = Units.inchesToMeters(43.51);

    public static final double UPRIGHT_HEIGHT = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double LOW_RUNG_HEIGHT = Units.inchesToMeters(27.0);
    public static final double MID_RUNG_HEIGHT = Units.inchesToMeters(45.0);
    public static final double HIGH_RUNG_HEIGHT = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d CENTER_POINT =
        new Translation2d(FRONT_FACE_X, FIELD_LAYOUT.getTagPose(31).get().getY());
    public static final Translation2d LEFT_UPRIGHT =
        new Translation2d(
            FRONT_FACE_X,
            FIELD_LAYOUT.getTagPose(31).get().getY()
                + INNER_OPENING_WIDTH / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d RIGHT_UPRIGHT =
        new Translation2d(
            FRONT_FACE_X,
            FIELD_LAYOUT.getTagPose(31).get().getY()
                - INNER_OPENING_WIDTH / 2
                - Units.inchesToMeters(0.75));

    // Relevant reference points on opposing side
    public static final Translation2d OPP_CENTER_POINT =
        new Translation2d(FIELD_LENGTH - FRONT_FACE_X, FIELD_LAYOUT.getTagPose(15).get().getY());
    public static final Translation2d OPP_LEFT_UPRIGHT =
        new Translation2d(
            FIELD_LENGTH - FRONT_FACE_X,
            FIELD_LAYOUT.getTagPose(15).get().getY()
                + INNER_OPENING_WIDTH / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d OPP_RIGHT_UPRIGHT =
        new Translation2d(
            FIELD_LENGTH - FRONT_FACE_X,
            FIELD_LAYOUT.getTagPose(15).get().getY()
                - INNER_OPENING_WIDTH / 2
                - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(42.0);
    public static final double DEPTH = Units.inchesToMeters(27.0);
    public static final double HEIGHT = Units.inchesToMeters(1.125);
    public static final double DISTANCE_FROM_CENTER_Y = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d DEPOT_CENTER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y, HEIGHT);
    public static final Translation3d LEFT_CORNER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y + (WIDTH / 2), HEIGHT);
    public static final Translation3d RIGHT_CORNER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y - (WIDTH / 2), HEIGHT);
  }

  public static class Outpost {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(31.8);
    public static final double OPENING_DISTANCE_FROM_FLOOR = Units.inchesToMeters(28.1);
    public static final double HEIGHT = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d CENTER_POINT =
        new Translation2d(0, FIELD_LAYOUT.getTagPose(29).get().getY());
  }

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
   * Creates a Vector from spherical coordinates.
   *
   * @param magnitude The magnitude of the vector.
   * @param direction The direction of the vector.
   * @return A Vector from the given spherical coordinates.
   */
  public static Vector<N3> fromSphericalCoords(double magnitude, Rotation3d direction) {
    double theta = direction.toRotation2d().getRadians();
    double alpha = direction.getY();

    return VecBuilder.fill(
        magnitude * Math.cos(theta) * Math.cos(-alpha),
        magnitude * Math.sin(theta) * Math.cos(-alpha),
        -magnitude * Math.sin(-alpha));
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
  public static Distance reflectDistance(Distance blueDist) {
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

  public static final Distance HUB_DIAMETER = Meters.of(1.059942);
  public static final Distance HUB_HEIGHT = Meters.of(1.8288);
  public static final Translation2d BLUE_HUB = new Translation2d(4.611624, 4.021328);
  public static final Translation2d RED_HUB = new Translation2d(11.901424, 4.021328);

  public static final double FUEL_MASS = 0.225;
  public static final double FUEL_RADIUS = 0.075;
}
