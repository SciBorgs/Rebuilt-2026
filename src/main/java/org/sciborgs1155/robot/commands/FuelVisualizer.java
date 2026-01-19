package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.robot.FieldConstants;

/**
 * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched from
 * the robot using the {@code shoot} command. This Fuel is reused upon the next calling of the
 * command. Fuel position can be accessed via the {@code getPose} method. All units are SI.
 */
public class FuelVisualizer {
  /** The index of the current {@code state} of the Fuel within the {@code trajectory} tensor. */
  private int index = 0;

  /**
   * A matrix whose row's represent the POSITION, VELOCITY, and ACCELERATION vectors of the Fuel
   * (using the field coordinate system).
   */
  private final double[][] state = new double[3][3];

  /**
   * A list of {@code state}'s (whose time-delta is specified by {@code FRAME_LENGTH}) specifying
   * the trajectory of the Fuel at the last call of the {@code shoot} method.
   */
  private final List<double[][]> trajectory = new ArrayList<>();

  /** The mass of the Fuel. */
  private static final double FUEL_MASS = 0.225;

  /**
   * The first, second, and third components of a 3D vector represent the X,Y, and Z coordinates
   * respectively. The positive Z axis faces to the ceiling. The positive X axis faces towards the
   * red alliance. The positive Y axis faces towards the left side of the blue alliance.
   */
  private static final int X = 0, Y = 1, Z = 2;

  /**
   * The first, second, and third components of the {@code state} matrix represent the POSITION,
   * VELOCITY, and ACCELERATION vectors respectively.
   */
  private static final int POSITION = 0, VELOCITY = 1, ACCELERATION = 2;

  /** The difference in time between {@code state}'s in the {@code trajectory}. */
  private static final double FRAME_LENGTH = 0.25;

  /** The dimensionless constant multiplied by velocity to attain the drag force. */
  private static final double DRAG_CONSTANT = 1;

  /** Once added to the pose of the robot, returns the pose of the shooter. */
  private static final Translation3d ROBOT_TO_SHOOTER = new Translation3d(0,0,1);

  /**
   * Acceleration due to gravity (GRAVITY FORCE = MASS * GRAVITY ACCELERATION).
   *
   * @return A vector representing the acceleration due to gravity.
   */
  private static final double[] getGravity() {
    return new double[] {0, 0, -9.81};
  }

  /**
   * Acceleration due to drag (DRAG FORCE = -VELOCITY * DRAG CONSTANT).
   *
   * @param velocity A vector representing the current velocity of the fuel.
   * @return A vector representing the acceleration due to drag.
   */
  private static final double[] getDrag(double[] velocity) {
    return scale(velocity, -(DRAG_CONSTANT / FUEL_MASS));
  }

  /**
   * Generates the trajectory of the Fuel after being launched with the provided parameters.
   *
   * @param startingPose The pose of the Fuel after losing contact with the shooter rollers
   *     utilizing the field coordinate system (X, Y, and Z).
   * @param startingVelocity The velocity of the Fuel after losing contact with the shooter rollers
   *     utilizing the field coordinate system (X, Y, and Z).
   * @return A list of {state}'s (whose time-delta is specified by {@code FRAME_LENGTH}) with
   *     POSITION, VELOCITY, AND ACCELERATION vectors (using the field coordinate system).
   */
  private static List<double[][]> generateTrajectory(
      double[] startingPose, double[] startingVelocity) {
    double[] pose = startingPose.clone();

    // VELOCITY (METERS / SECOND) --> VELOCITY (METERS / FRAME)
    double[] velocity = scale(startingVelocity.clone(), FRAME_LENGTH);

    // TRAJECTORY GENERATION
    List<double[][]> trajectory = new ArrayList<>();

    while (inField(pose)) {
      // ACCELERATION = GRAVITY + DRAG (METERS / SECOND^2)
      double[] acceleration = sum(getGravity(), getDrag(velocity));

      // VELOCITY = ACCELERATION * TIME (METERS / FRAME)
      addTo(velocity, scale(acceleration, FRAME_LENGTH));

      // DISPLACEMENT = VELOCITY * TIME (METERS)
      addTo(pose, velocity);

      // ADD STATE TO TRAJECTORY (CONVERT VELOCITY TO METERS / SECOND)
      trajectory.add(new double[][] {pose, scale(velocity, 1 / FRAME_LENGTH), acceleration});
    }

    return trajectory;
  }

  public Command shoot(
      Supplier<LinearVelocity> launchVelocity,
      Supplier<Angle> turretAngle,
      Supplier<Angle> hoodAngle,
      Supplier<Pose3d> robotPose) {

    // COMMAND GENERATION
    return Commands.runOnce(
            () -> {
              reset();
              init(launchVelocity.get(), turretAngle.get(), hoodAngle.get(), robotPose.get());
            })
        .andThen(
            Commands.repeatingSequence(
                    Commands.runOnce(this::next).andThen(Commands.waitSeconds(FRAME_LENGTH)))
                .until(() -> index >= trajectory.size()))
        .finallyDo(this::reset);
  }

  /**
   * Generates the trajectory of the Fuel.
   *
   * @param velocity The launch velocity of the Fuel.
   * @param yaw The yaw of the turret.
   * @param pitch The pitch of the hood.
   * @param robotPose The current position of the robot.
   */
  private void init(LinearVelocity velocity, Angle yaw, Angle pitch, Pose3d robotPose) {
    Translation3d shooterPose = robotPose.getTranslation().plus(ROBOT_TO_SHOOTER);

    double[] startingPose = {shooterPose.getX(), shooterPose.getY(), shooterPose.getZ()};
    double[] startingVelocity =
        fromSpherical(
            velocity.in(MetersPerSecond), yaw.in(Radians), pitch.in(Radians) - Math.PI / 2);

    for (double[][] generatedState : generateTrajectory(startingPose, startingVelocity))
      trajectory.add(generatedState);
  }

  /** Sets all Fuel properties to the default setting. */
  private void reset() {
    trajectory.clear();
    state[POSITION] = new double[3];
    state[VELOCITY] = new double[3];
    state[ACCELERATION] = new double[3];
    index = 0;
  }

  /** Displays the next {@code state} in the {@code trajectory}. To be called periodically. */
  private void next() {
    double[][] generatedState = trajectory.get(index);
    state[POSITION] = generatedState[POSITION];
    state[VELOCITY] = generatedState[VELOCITY];
    state[ACCELERATION] = generatedState[ACCELERATION];
    index++;
  }

  /**
   * The current position of the Fuel on the field.
   *
   * @return The pose of the Fuel (using the field coordinate system).
   */
  @Logged(name = "FUEL POSE")
  public Pose3d getPose() {
    return new Pose3d(state[POSITION][X], state[POSITION][Y], state[POSITION][Z], Rotation3d.kZero);
  }

  /**
   * The current velocity of the Fuel.
   *
   * @return The velocity of the Fuel (using the field coordinate system).
   */
  @Logged(name = "FUEL VELOCITY")
  public Translation3d getVelocity() {
    return new Translation3d(state[VELOCITY][X], state[VELOCITY][Y], state[VELOCITY][Z]);
  }

  /**
   * The current acceleration of the Fuel.
   *
   * @return The acceleration of the Fuel (using the field coordinate system).
   */
  @Logged(name = "FUEL ACCELERATION")
  public Translation3d getAcceleration() {
    return new Translation3d(
        state[ACCELERATION][X], state[ACCELERATION][Y], state[ACCELERATION][Z]);
  }

  /**
   * The timestamp of the latest launch.
   *
   * @return The time since the last launch of the fuel.
   */
  @Logged(name = "TIMESTAMP")
  public Time getTimestamp() {
    return Seconds.of(index * FRAME_LENGTH);
  }

  /**
   * Adds all elements of the added vector into the original vector.
   *
   * @param original The vector to be added to.
   * @param addend The vector to add to the original.
   */
  private static void addTo(double[] original, double[] addend) {
    if (original.length != addend.length)
      throw new UnsupportedOperationException("Arrays must be of equal lengths!");
    for (int index = 0; index < original.length; index++) original[index] += addend[index];
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
   * Converts spherical coordinates to cartesian coordinates.
   *
   * @param magnitude The magnitude of the polar coordinate.
   * @param theta The angle above the positive z-axis in radians.
   * @param alpha The angle to the left of the positive x-axis in radians.
   * @return A vector representing the cartesian coordinates.
   */
  private static double[] fromSpherical(double magnitude, double theta, double alpha) {
    return scale(
        new double[] {
          Math.sin(theta) * Math.cos(alpha), Math.sin(theta) * Math.sin(alpha), Math.cos(theta)
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
    if (pose[Z] < 0) return false;

    return true;
  }
}
