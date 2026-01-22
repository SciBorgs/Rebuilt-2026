package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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

/** Simulates the behavior multiple {@code Fuel} projectiles using {@code FuelSim}. */
public final class FuelVisualizer {
  /** Not meant to be used. */
  private FuelVisualizer() {}

  /** All Fuel currently being simulated on the field. */
  private static List<FuelSim> fuelSims = new ArrayList<>();

  /** A supplier for the launch velocity of the Fuel. */
  private static Supplier<LinearVelocity> launchVelocitySupplier;

  /** A supplier for the angle of the turret. */
  private static Supplier<Angle> turretAngleSupplier;

  /** A supplier for the angle of the hood. */
  private static Supplier<Angle> hoodAngleSupplier;

  /** A supplier for the current pose of the robot. */
  private static Supplier<Pose3d> robotPoseSupplier;

  /**
   * A publisher for the positions of the {@code FuelSim}'s. Used to view Fuel in logging framework.
   */
  private static StructArrayPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("Visualizer", Pose3d.struct).publish();

  /**
   * Initializes suppliers for Fuel launch parameters. To be called on robot startup.
   *
   * @param launchVelocitySupplier A supplier for the launch velocity of the Fuel.
   * @param turretAngleSupplier A supplier for the angle of the turret.
   * @param hoodAngleSupplier A supplier for the angle of the hood.
   * @param robotPoseSupplier A supplier for the pose of the robot.
   */
  public static void init(
      Supplier<LinearVelocity> launchVelocitySupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier,
      Supplier<Pose3d> robotPoseSupplier) {
    FuelVisualizer.launchVelocitySupplier = launchVelocitySupplier;
    FuelVisualizer.turretAngleSupplier = turretAngleSupplier;
    FuelVisualizer.hoodAngleSupplier = hoodAngleSupplier;
    FuelVisualizer.robotPoseSupplier = robotPoseSupplier;
  }

  /**
   * Adds {@code FuelSim}'s to the visualizer.
   *
   * @param quantity The amount of {@code FuelSim}'s to add.
   */
  public static void addFuel(int quantity) {
    for (int index = 0; index < quantity; index++)
      fuelSims.add(
          new FuelSim(
              launchVelocitySupplier, turretAngleSupplier, hoodAngleSupplier, robotPoseSupplier));
  }

  /**
   * Shoots the first idle {@code FuelSim} in the visualizer. If no idle Fuel are present, the
   * earliest launched Fuel is used.
   *
   * @return A command to launch Fuel.
   */
  public static Command shootFuel() {
    return getLaunchableFuel().shoot();
  }

  /** Publishes the current poses of the {@code FuelSim}'s to {@code NetworkTables}. */
  public static void update() {
    Pose3d[] poses = new Pose3d[fuelSims.size()];
    for (int index = 0; index < poses.length; index++) poses[index] = fuelSims.get(index).getPose();
    publisher.accept(poses);
  }

  /**
   * Indexes and returns the first idle {@code FuelSim} in the visualizer. If no idle Fuel are
   * present, the earliest launched Fuel is used.
   *
   * @return The first idle Fuel in the visualizer.
   */
  private static FuelSim getLaunchableFuel() {
    for (FuelSim fuelSim : fuelSims) if (fuelSim.isIdle()) return fuelSim;
    return fuelSims.get(0);
  }

  /**
   * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched from
   * the robot using the {@code shoot} command. This Fuel is reused upon the next calling of the
   * command. Fuel position can be accessed via the {@code getPose} method. All units are SI unless
   * specified otherwise.
   */
  public static class FuelSim {
    /** The index of the current {@code state} of the Fuel within the {@code trajectory} tensor. */
    private int frame;

    /** Whether or not the Fuel is currently being launched. */
    private boolean idle = true;

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

    /** A supplier for the launch velocity of the Fuel. */
    private final Supplier<LinearVelocity> launchVelocity;

    /** A supplier for the angle of the turret. */
    private final Supplier<Angle> turretAngle;

    /** A supplier for the angle of the hood. */
    private final Supplier<Angle> hoodAngle;

    /** A supplier for the current pose of the robot. */
    private final Supplier<Pose3d> robotPose;

    /**
     * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched
     * from the robot using the {@code shoot} command. This Fuel is reused upon the next calling of
     * the command. Fuel position can be accessed via the {@code getPose} method. All units are SI
     * unless specified otherwise.
     *
     * @param launchVelocity A supplier for the launch velocity of the Fuel.
     * @param turretAngle A supplier for the angle of the turret.
     * @param hoodAngle A supplier for the angle of the hood.
     * @param robotPose A supplier for the pose of the robot.
     * @param startingPose The starting position of the Fuel.
     */
    public FuelSim(
        Supplier<LinearVelocity> launchVelocity,
        Supplier<Angle> turretAngle,
        Supplier<Angle> hoodAngle,
        Supplier<Pose3d> robotPose) {
      this.launchVelocity = launchVelocity;
      this.turretAngle = turretAngle;
      this.hoodAngle = hoodAngle;
      this.robotPose = robotPose;
    }

    /**
     * Resets and launches Fuel from the robot.
     *
     * @return A command to launch the Fuel from the robot.
     */
    public Command shoot() {
      return Commands.runOnce(this::init)
          .andThen(
              Commands.repeatingSequence(
                      Commands.runOnce(this::next).andThen(Commands.waitSeconds(FRAME_LENGTH)))
                  .onlyWhile(() -> frame < trajectory.size()))
          .finallyDo(this::end);
    }

    /** Resets {@code frame} and generates the new trajectory for the Fuel. */
    private void init() {
      // RESET
      frame = 0;
      idle = false;
      trajectory.clear();

      // TRAJECTORY GENERATION
      double[] startingPose =
          getFuelStartingPose(robotPose.get(), turretAngle.get(), hoodAngle.get());
      double[] startingVelocity =
          getFuelStartingVelocity(launchVelocity.get(), turretAngle.get(), hoodAngle.get());

      for (double[][] generatedState : generateTrajectory(startingPose, startingVelocity))
        trajectory.add(generatedState);
    }

    /** Displays the next {@code state} in the {@code trajectory}. To be called periodically. */
    private void next() {
      if (frame >= trajectory.size()) return;
      double[][] generatedState = trajectory.get(frame);
      state[POSITION] = generatedState[POSITION];
      state[VELOCITY] = generatedState[VELOCITY];
      state[ACCELERATION] = generatedState[ACCELERATION];
      frame++;
    }

    /** Called when Fuel completes it's trajectory. */
    private void end() {
      idle = true;
    }

    /**
     * The current pose of the Fuel on the field.
     *
     * @return The pose of the Fuel (using the field coordinate system).
     */
    @Logged(name = "FUEL POSE")
    public Pose3d getPose() {
      return new Pose3d(
          state[POSITION][X], state[POSITION][Y], state[POSITION][Z], Rotation3d.kZero);
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
      return Seconds.of(frame * FRAME_LENGTH);
    }

    /**
     * Whether or not the Fuel is currently being launched.
     *
     * @return True if it is not being launched, False if it is being launched.
     */
    @Logged(name = "IDLE")
    public boolean isIdle() {
      return idle;
    }
  }

  /** The mass of the Fuel (kilograms). */
  private static final double FUEL_MASS = 0.225;

  /** The diameter of the Fuel (meters). */
  private static final double FUEL_DIAMETER = 0.15;

  /**
   * The first, second, and third components of a 3D vector represent the X,Y, and Z coordinates
   * respectively. The positive Z axis faces to the ceiling. The positive X axis faces towards the
   * red alliance. The positive Y axis faces towards the left side of the blue alliance.
   */
  @SuppressWarnings("PMD.OneDeclarationPerLine")
  private static final int X = 0, Y = 1, Z = 2;

  /**
   * The first, second, and third components of the {@code state} matrix represent the POSITION,
   * VELOCITY, and ACCELERATION vectors respectively.
   */
  @SuppressWarnings("PMD.OneDeclarationPerLine")
  private static final int POSITION = 0, VELOCITY = 1, ACCELERATION = 2;

  /** The difference in time between {@code state}'s in the {@code trajectory}. */
  private static final double FRAME_LENGTH = 0.016;

  /** The dimensionless constant multiplied by velocity to attain the drag force. */
  private static final double DRAG_CONSTANT = 0.1;

  /** Once added to the pose of the robot, returns the pose of the shooter. */
  private static final Translation3d ROBOT_TO_SHOOTER = new Translation3d(0, 0, 0.1);

  /**
   * Acceleration due to gravity (GRAVITY FORCE = MASS * GRAVITY ACCELERATION).
   *
   * @return A vector representing the acceleration due to gravity (METERS / FRAME^2).
   */
  private static double[] getGravity() {
    return new double[] {0, 0, -9.81 * FRAME_LENGTH};
  }

  /**
   * Acceleration due to drag (DRAG FORCE = -VELOCITY * DRAG CONSTANT).
   *
   * @param velocity A vector representing the current velocity of the fuel.
   * @return A vector representing the acceleration due to drag (METERS / FRAME^2).
   */
  private static double[] getDrag(double[] velocity) {
    return scale(velocity, FRAME_LENGTH * -(DRAG_CONSTANT / FUEL_MASS));
  }

  /**
   * Generates the starting pose vector for the Fuel based on robot properties.
   *
   * @param yaw The yaw of the turret.
   * @param pitch The pitch of the hood.
   * @param robotPose The current position of the robot.
   */
  private static double[] getFuelStartingPose(Pose3d robotPose, Angle yaw, Angle pitch) {
    // ORIGIN OF THE SHOOTER
    Translation3d shooterPose = robotPose.getTranslation().plus(ROBOT_TO_SHOOTER);
    double[] startingPose = {shooterPose.getX(), shooterPose.getY(), shooterPose.getZ()};

    // TRANSFORM BASED ON TURRET / HOOD DIRECTION.
    double[] shooterToFuel = toCartesian(0.2 + FUEL_DIAMETER, yaw.in(Radians), pitch.in(Radians));

    addTo(startingPose, shooterToFuel);
    return startingPose;
  }

  /**
   * Generates the starting velocity vector for the Fuel based on robot properties.
   *
   * @param velocity The launch velocity of the Fuel.
   * @param yaw The yaw of the turret.
   * @param pitch The pitch of the hood.
   */
  private static double[] getFuelStartingVelocity(LinearVelocity velocity, Angle yaw, Angle pitch) {
    return toCartesian(velocity.in(MetersPerSecond), yaw.in(Radians), pitch.in(Radians));
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

    // ACCELERATION (METERS / FRAME^2)
    double[] acceleration = new double[3];

    // TRAJECTORY GENERATION
    List<double[][]> trajectory = new ArrayList<>();

    while (inField(pose)) {
      // ADD STATE TO TRAJECTORY (CONVERT VELOCITY AND ACCELERATION TO METERS / SECOND)
      trajectory.add(
          new double[][] {
            scale(pose, 1), scale(velocity, 1 / FRAME_LENGTH), scale(acceleration, 1 / FRAME_LENGTH)
          });

      // ACCELERATION = GRAVITY + DRAG (METERS / FRAME^2)
      acceleration = sum(getGravity(), getDrag(velocity));

      // VELOCITY = ACCELERATION * TIME (METERS / FRAME)
      addTo(velocity, acceleration);

      // DISPLACEMENT = VELOCITY * TIME (METERS)
      addTo(pose, velocity);
    }

    // PREVENTS CLIPPING OUT OF FRAME
    double[] finalPose = pose.clone();

    finalPose[X] = MathUtil.clamp(pose[X], 0, FieldConstants.LENGTH.in(Meters));
    finalPose[Y] = MathUtil.clamp(pose[Y], 0, FieldConstants.WIDTH.in(Meters));
    finalPose[Z] = Math.max(0, pose[Z]);

    trajectory.add(
        new double[][] {
          finalPose, scale(velocity, 1 / FRAME_LENGTH), scale(acceleration, 1 / FRAME_LENGTH)
        });

    return trajectory;
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
   * @param theta The angle in the xy-plane (polar angle).
   * @param alpha The angle up from the xy-plane (towards positive z).
   * @return A vector representing the cartesian coordinates.
   */
  private static double[] toCartesian(double magnitude, double theta, double alpha) {
    return new double[] {
      magnitude * Math.cos(theta) * Math.cos(alpha),
      magnitude * Math.sin(theta) * Math.cos(alpha),
      -magnitude * Math.sin(alpha)
    };
  }

  /**
   * Determines if the specified object is within the bounds of the field. Utilizes the field
   * coordinate system.
   *
   * @param pose The pose of the object to evaluate.
   * @return True if the pose is within the field's bounds. False if otherwise.
   */
  @SuppressWarnings("PMD.AvoidLiteralsInIfCondition")
  private static boolean inField(double[] pose) {
    if (pose.length != 3) throw new InvalidParameterException("Pose must be a 3D vector!");
    return pose[X] > 0
        && pose[Y] > 0
        && pose[X] < FieldConstants.LENGTH.in(Meters)
        && pose[Y] < FieldConstants.WIDTH.in(Meters)
        && pose[Z] > 0;
  }
}
