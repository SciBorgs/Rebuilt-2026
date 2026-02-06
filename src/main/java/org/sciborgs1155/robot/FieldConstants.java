package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.alliance;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldConstants {
  // Origin at corner of blue alliance side of field
  public static final Distance LENGTH = Centimeters.of(1755);
  public static final Distance WIDTH = Centimeters.of(805);

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
