package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.FieldConstants;

/**
 * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched from
 * the robot using the {@code shoot} command. This Fuel is reused upon the next calling of the
 * command. Fuel position can be accessed via the {@code getPose} method. All units are SI.
 */
public class FuelVisualizer {
  /**
   * The first, second, and third components of a 3D vector represent the X,Y, and Z coordinates
   * respectively. The positive Z axis faces to the ceiling.
   */
  private static final int X = 0, Y = 1, Z = 2;

  /**
   * The first, second, and third components of the timestamp matrix represent the POSITION,
   * VELOCITY, and ACCELERATION vectors respectively.
   */
  private static final int P = 0, V = 1, A = 2;

  /**
   * The amount of trajectory-frames generated per second. A larger value will be more
   * computationally intensive, however it will be more accurate.
   */
  private static final int RESOLUTION = 500;

  /**
   * A vector representing the X, Y, and Z components of the Fuel's pose (using the field coordinate
   * system).
   */
  private static final double[] fuelPose = new double[3];

  /**
   * A vector representing the X, Y, and Z components of the Fuel's velocity (using the field
   * coordinate system).
   */
  private static final double[] fuelVelocity = new double[3];

  /**
   * A vector representing the X, Y, and Z components of the Fuel's acceleration (using the field
   * coordinate system).
   */
  private static final double[] fuelAcceleration = new double[3];

  /** The index of the timestamp of the trajectory of the fuel (what a mouthful!). */
  private static int index = 0;

  /** The time, in seconds, since the last launch of the fuel. */
  private static double timestamp = 0;

  /** Acceleration due to drag. */
  private static final double[] drag = {0, 0, 0};

  /** The mass of the Fuel. */
  private static final double mass = 0.225;

  /** Acceleration due to gravity. */
  private static final double[] gravity = {0, 0, -9.81};

  /**
   * The dimensionless constant multiplied by velocity to attain the magnitude of the drag force
   * applied to the Fuel.
   */
  private static final double dragConstant = 1;

  /** A transform that, once applied to the pose of the robot, returns the pose of the shooter. */
  private static final Transform3d robotToShooter = new Transform3d();

  /**
   * Generates the trajectory of the Fuel after being launched with the provided parameters.
   *
   * @param startingPose The pose of the Fuel after losing contact with the shooter rollers
   *     utilizing the field coordinate system (X, Y, and Z).
   * @param startingVelocity The velocity of the Fuel after losing contact with the shooter rollers
   *     utilizing the field coordinate system (X, Y, and Z).
   * @param startingAcceleration The acceleration of the Fuel after losing contact with the shooter
   *     rollers utilizing the field coordinate system (X, Y, and Z).
   * @return A list of timestamps (whose time-delta is specified by {@code RESOLUTION}) with
   *     POSITION, VELOCITY, AND ACCELERATION vectors (using the field coordinate system).
   */
  private static List<double[][]> generateTrajectory(
      double[] startingPose, double[] startingVelocity, double[] startingAcceleration) {
    // INPUT VALIDATION
    if (startingPose.length != 3)
      throw new InvalidParameterException("Starting pose must be a 3D vector!");
    if (startingVelocity.length != 3)
      throw new InvalidParameterException("Starting velocity must be a 3D vector!");
    if (startingAcceleration.length != 3)
      throw new InvalidParameterException("Starting acceleration must be a 3D vector!");

    // STARTING STATE
    final List<double[][]> trajectory = new ArrayList<>();

    // UNITS ARE METERS / FRAME
    final double[] generatedPose = new double[3];
    setTo(generatedPose, startingPose);

    final double[] generatedVelocity = new double[3];
    setTo(generatedVelocity, scale(startingVelocity, 1 / RESOLUTION));

    final double[] generatedAcceleration = new double[3];
    setTo(generatedAcceleration, scale(startingAcceleration, 1 / RESOLUTION));

    // TRAJECTORY GENERATION
    while (inField(generatedPose)) {
      // ADD TIMESTAMP
      trajectory.add(
          new double[][] {
            generatedPose,
            scale(generatedVelocity, RESOLUTION),
            scale(generatedAcceleration, RESOLUTION)
          });

      // APPLY VELOCITY
      setTo(generatedPose, sum(generatedPose, generatedVelocity));

      // APPLY FORCE
      setTo(drag, scale(negate(generatedVelocity), dragConstant / mass));
      setTo(generatedAcceleration, sum(scale(gravity, 1 / RESOLUTION), drag));

      // APPLY ACCELERATION
      setTo(generatedVelocity, sum(generatedVelocity, generatedAcceleration));
    }

    return trajectory;
  }

  public static Command shoot(
      LinearVelocity launchVelocity, Angle turretAngle, Angle hoodAngle, Pose3d robotPose) {
    // CONVERT ROBOT COORDINATES TO SHOOTER COORDINATES
    final Translation3d shooterPose = robotPose.transformBy(robotToShooter).getTranslation();
    final Rotation3d shooterRotation =
        robotPose.getRotation().rotateBy(new Rotation3d(Radians.zero(), hoodAngle, turretAngle));

    // STARTING STATE
    index = 0;
    timestamp = 0;

    final double[] startingPose = {shooterPose.getX(), shooterPose.getY(), shooterPose.getZ()};
    final double[] startingVelocity =
        fromPolar(
            launchVelocity.in(MetersPerSecond), shooterRotation.getY(), shooterRotation.getZ());

    // TRAJECTORY GENERATION
    final List<double[][]> trajectory =
        generateTrajectory(startingPose, startingVelocity, new double[3]);

    // COMMAND GENERATION
    return Commands.repeatingSequence(
        Commands.runOnce(
                () -> {
                  final double[][] state = trajectory.get(index);

                  setTo(fuelPose, state[P]);
                  setTo(fuelVelocity, state[V]);
                  setTo(fuelAcceleration, state[A]);

                  timestamp = index / RESOLUTION;
                  index++;
                })
            .andThen(Commands.waitSeconds(1 / RESOLUTION)));
  }

  /**
   * Copies all elements of the replacement vector into the original vector.
   *
   * @param original The vector to be replaced.
   * @param replacement The vector to be the replacement.
   */
  private static void setTo(double[] original, double[] replacement) {
    if (original.length != replacement.length)
      throw new UnsupportedOperationException("Arrays must be of equal lengths!");
    for (int index = 0; index < original.length; index++) original[index] = replacement[index];
  }

  /**
   * Adds each element of the first vector to the corresponding element of the second vector.
   *
   * @param vector1 The first addend vector.
   * @param vector2 The second addend vector.
   * @return A new vector containing the sum of the vectors.
   */
  private static double[] sum(double[] vector1, double[] vector2) {
    if (vector1.length != vector2.length)
      throw new UnsupportedOperationException("Arrays must be of equal lengths!");
    double[] sum = new double[vector1.length];
    for (int index = 0; index < vector1.length; index++)
      sum[index] = vector1[index] + vector2[index];
    return sum;
  }

  /**
   * Negates all elements of the vector. Original vector remains unmodified.
   *
   * @param vector The vector to negate.
   * @return A new vector with the elements negated.
   */
  private static double[] negate(double[] vector) {
    double[] negation = new double[vector.length];
    for (int index = 0; index < vector.length; index++) negation[index] = -vector[index];
    return negation;
  }

  /**
   * Scales all elements of the vector by the specified scalar. Original vector remains unmodified.
   *
   * @param vector The vector to scale.
   * @param scalar The scalar to scale by.
   * @return A new vector with the scaled elements.
   */
  private static double[] scale(double[] vector, double scalar) {
    double[] scaled = new double[vector.length];
    for (int index = 0; index < vector.length; index++) scaled[index] = vector[index] * scalar;
    return scaled;
  }

  /**
   * Converts polar coordinates to cartesian coordinates.
   *
   * @param magnitude The magnitude of the polar coordinate.
   * @param theta The angle above the positive z-axis.
   * @param alpha The angle to the left of the positive x-axis.
   * @return A vector representing the cartesian coordinates.
   */
  private static double[] fromPolar(double magnitude, double theta, double alpha) {
    return scale(
        new double[] {
          Math.sin(theta) * (Math.cos(alpha)), Math.sin(theta) * (Math.sin(alpha)), Math.cos(theta)
        },
        magnitude);
  }

  /**
   * Determines if the specified object is within the bounds of the field. Utilizes the field
   * coordinate system.
   *
   * @param pose The pose of the object to evaluate.
   * @return True if the pose is within the field's bounds. False if otherwise.
   */
  private static boolean inField(double[] pose) {
    if (pose.length != 3) throw new InvalidParameterException("pose must be a 3D vector!");
    if (pose[X] <= 0 || pose[Y] <= 0) return false;
    if (pose[X] >= FieldConstants.LENGTH.in(Meters)) return false;
    if (pose[Y] >= FieldConstants.WIDTH.in(Meters)) return false;

    return true;
  }

  /**
   * The current position of the Fuel on the field.
   *
   * @return The pose of the Fuel (using the field coordinate system).
   */
  @Logged(name = "FUEL POSE")
  public static Pose3d getPose() {
    return new Pose3d(fuelPose[X], fuelPose[Y], fuelPose[Z], Rotation3d.kZero);
  }

  /**
   * The current velocity of the Fuel.
   *
   * @return The velocity of the Fuel (using the field coordinate system).
   */
  @Logged(name = "FUEL VELOCITY")
  public static Translation3d getVelocity() {
    return new Translation3d(fuelVelocity[X], fuelVelocity[Y], fuelVelocity[Z]);
  }

  /**
   * The current acceleration of the Fuel.
   *
   * @return The acceleration of the Fuel (using the field coordinate system).
   */
  @Logged(name = "FUEL ACCELERATION")
  public static Translation3d getAcceleration() {
    return new Translation3d(fuelAcceleration[X], fuelAcceleration[Y], fuelAcceleration[Z]);
  }

  /**
   * The timestamp of the latest launch.
   *
   * @return The time since the last launch of the fuel.
   */
  @Logged(name = "TIMESTAMP")
  public static Time getTimestamp() {
    return Seconds.of(timestamp);
  }
}
