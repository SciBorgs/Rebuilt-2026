package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.sciborgs1155.robot.FieldConstants;

/** Simulates the behavior of multiple Fuel projectiles using {@code FuelSim}. */
public final class FuelVisualizer {
  /** The mass of the Fuel (KILOGRAMS). */
  private static final double FUEL_MASS = 0.225;

  /** Force due to gravity (NEWTONS). */
  private static final double GRAVITY = -9.81 * FUEL_MASS;

  /** The dimensionless constant multiplied by velocity to attain the drag force. */
  private static final double DRAG_CONSTANT = -0.47;

  /** The diameter of the Fuel (METERS). */
  private static final double FUEL_DIAMETER = 0.15;

  /** How fast the Fuel spins in the air (RADIANS / SECOND). Purely cosmetic. */
  private static final double SPIN = 1.0;

  /** This constructor is not meant to be used. */
  private FuelVisualizer() {}

  /** The index of the latest Fuel to be launched. */
  private static int fuelIndex;

  /** All Fuel currently being simulated. */
  private static FuelSim[] fuelSims;

  /** The poses of every Fuel currently being simulated. */
  private static Pose3d[] fuelPoses;

  /** The idle states of every Fuel currently being simulated. */
  private static boolean[] fuelStates;

  /** A supplier for the launch velocity of the Fuel. */
  private static Supplier<AngularVelocity> shooterVelocity;

  /** A supplier for the angle of the turret. */
  private static Supplier<Angle> turretAngle;

  /** A supplier for the angle of the hood. */
  private static Supplier<Angle> hoodAngle;

  /** A supplier for the current pose of the robot. */
  private static Supplier<Pose3d> robotPose;

  /** The length of each frame in the Fuel launch animation (SECONDS / FRAME). */
  private static final double FRAME_LENGTH = 0.02;

  /**
   * The ratio between the angular velocity of the shooter (RADIANS / SECOND) and the launch
   * velocity of the Fuel (METERS / SECOND).
   */
  private static final double SHOOTER_TO_LAUNCH_VELOCITY = 1;

  /**
   * The distance between the origin of the shooter and the actual launch position of the Fuel
   * (METERS).
   */
  private static final double SHOOTER_TO_FUEL = 0;

  /** Once added to the pose of the robot, returns the pose of the shooter (METERS). */
  private static final Vector<N3> ROBOT_TO_SHOOTER = VecBuilder.fill(0, 0, 1);

  /**
   * A publisher for the positions of the {@code FuelSim}'s. Used to track Fuel in logging
   * framework.
   */
  private static StructArrayPublisher<Pose3d> fuelPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Fuel Visualizer/Fuel Poses", Pose3d.struct)
          .publish();

  /**
   * A publisher for the idle states of the {@code FuelSim}'s. Used to track Fuel in logging
   * framework.
   */
  private static BooleanArrayPublisher fuelStatePublisher =
      NetworkTableInstance.getDefault()
          .getBooleanArrayTopic("Fuel Visualizer/Fuel States")
          .publish();

  /**
   * To be called on robot startup. Parameters used to calculate Fuel trajectory after launch.
   *
   * @param shooterVelocitySupplier A supplier for the angular velocity of the {@code Shooter}.
   * @param turretAngleSupplier A supplier for the angle of the {@code Turret}.
   * @param hoodAngleSupplier A supplier for the angle of the {@code Hood}.
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param fuelCapacity The number of Fuel's to simulate.
   */
  public static void init(
      Supplier<AngularVelocity> shooterVelocitySupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier,
      Supplier<Pose3d> robotPoseSupplier,
      int fuelCapacity) {
    shooterVelocity = shooterVelocitySupplier;
    turretAngle = turretAngleSupplier;
    hoodAngle = hoodAngleSupplier;
    robotPose = robotPoseSupplier;

    // FUEL INSTANTIATION
    fuelSims = new FuelSim[fuelCapacity];
    fuelPoses = new Pose3d[fuelCapacity];
    fuelStates = new boolean[fuelCapacity];

    // INSTANTIATION
    for (int index = 0; index < fuelSims.length; index++) {
      fuelSims[index] = new FuelSim();
      fuelPoses[index] = new Pose3d();
      fuelStates[index] = false;
    }
  }

  /**
   * Launches the first idle Fuel in the visualizer. If no idle Fuel are present, the earliest
   * launched Fuel is used.
   *
   * @return A command to launch Fuel.
   */
  public static Command launchFuel() {
    return Commands.deferredProxy(() -> Commands.runOnce(getLaunchableFuel()::init));
  }

  /** Publishes the Fuel display data to {@code NetworkTables}. */
  public static void periodic() {
    int index = 0;

    // UPDATING SIMULATIONS
    for (FuelSim fuelSim : fuelSims) {
      fuelSim.nextFrame();
      fuelPoses[index] = getFuelPose3d(fuelSim.translation, fuelSim.rotation);
      fuelStates[index] = fuelSim.isBeingLaunched;
      index++;
    }

    // PUBLISHING DATA
    fuelPosePublisher.accept(fuelPoses);
    fuelStatePublisher.accept(fuelStates);
  }

  /**
   * Returns the first un-launched Fuel. If all Fuel has been launched, return the first idle Fuel.
   * If no Fuel meets this criteria, return the first Fuel in the visualizer.
   *
   * @return The first launchable Fuel.
   */
  private static FuelSim getLaunchableFuel() {
    if (fuelIndex >= fuelSims.length) fuelIndex = 0;
    boolean fullCycle = fuelIndex == 0;

    // ITERATE THROUGH NON-CYCLED FUEL
    for (int index = fuelIndex + 1; index < fuelSims.length; index++)
      if (!fuelSims[index].isBeingLaunched) {
        fuelIndex = index;
        return fuelSims[index];
      }

    if (fullCycle) return fuelSims[0];

    // RECYCLE
    fuelIndex = 0;
    return getLaunchableFuel();
  }

  /**
   * Generates the launch translation vector for the Fuel based on robot properties.
   *
   * @return A vector representing the translation of the Fuel at launch (METERS).
   */
  private static Vector<N3> getFuelStartingTranslation() {
    Vector<N3> shooterOrigin = robotPose.get().getTranslation().toVector().plus(ROBOT_TO_SHOOTER);
    Vector<N3> shooterToFuel =
        FieldConstants.fromSphericalCoords(
            SHOOTER_TO_FUEL + FUEL_DIAMETER,
            turretAngle.get().in(Radians),
            hoodAngle.get().in(Radians));

    return shooterOrigin.plus(shooterToFuel);
  }

  /**
   * Generates the launch velocity vector for the Fuel based on robot properties.
   *
   * @return A vector representing the translation of the Fuel at launch (METERS / SECOND).
   */
  private static Vector<N3> getFuelStartingVelocity() {
    return FieldConstants.fromSphericalCoords(
        shooterVelocity.get().in(RadiansPerSecond) * SHOOTER_TO_LAUNCH_VELOCITY,
        turretAngle.get().in(Radians),
        hoodAngle.get().in(Radians)).times(FRAME_LENGTH);
  }

  /**
   * Converts translation and rotation vectors of the Fuel into a pose object.
   *
   * @return The pose of the Fuel (METERS).
   */
  private static Pose3d getFuelPose3d(Vector<N3> translation, Vector<N3> rotation) {
    return new Pose3d(new Translation3d(translation), new Rotation3d(rotation));
  }

  /**
   * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched from
   * the robot using the {@code shoot} command. This Fuel is reused upon the next calling of the
   * command. All units are SI unless specified otherwise.
   */
  public static class FuelSim {
    /** If the Fuel is suspended in the air, it is being launched. */
    protected boolean isBeingLaunched;

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's
     * translation (METERS).
     */
    protected Vector<N3> translation = VecBuilder.fill(0, 0, 0);

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's rotation
     * (RADIANS).
     */
    protected Vector<N3> rotation = VecBuilder.fill(0, 0, 0);

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's velocity
     * (METERS / FRAME).
     */
    protected Vector<N3> velocity = VecBuilder.fill(0, 0, 0);

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's acceleration (METERS / FRAME^2).
     */
    protected Vector<N3> acceleration = VecBuilder.fill(0,0,0);

    /** Starts frame generation. */
    private void init() {
      if (isBeingLaunched) return;

      translation = getFuelStartingTranslation();
      velocity = getFuelStartingVelocity();
      acceleration = VecBuilder.fill(0,0,0);
      rotation = VecBuilder.fill(0, 0, 0);

      // ALLOWS FRAMES TO BE RENDERED
      isBeingLaunched = true;
    }

    /** Generates a new frame. To be called periodically. */
    private void nextFrame() {
      if (!isBeingLaunched) return;

      // SUM OF FORCES IS GRAVITY + DRAG
      acceleration.setColumn(0, VecBuilder.fill(0, 0, GRAVITY).plus(velocity.times(DRAG_CONSTANT)));

      // CONVERT FORCE (IN SECONDS) TO ACCELERATION (IN FRAMES)
      acceleration.setColumn(0, acceleration.times(FRAME_LENGTH / FUEL_MASS));

      // ADD ACCELERATION TO VELOCITY
      velocity.setColumn(0, velocity.plus(acceleration));

      // ADD VELOCITY TO TRANSLATION
      translation.setColumn(0, translation.plus(velocity));

      // SPIN IS PURELY COSMETIC (FOR NOW)
      rotation.setColumn(0, rotation.plus(SPIN).times(FRAME_LENGTH));

      // IF THE FUEL TOUCHES THE GROUND, FRAME GENERATION ENDS
      if (translation.get(2) < FUEL_DIAMETER / 2) end();
    }

    /** Ends frame generation. */
    private void end() {
      // PREVENT CLIPPING INTO THE GROUND
      translation.set(2, 0, FUEL_DIAMETER / 2);
      rotation = VecBuilder.fill(0, 0, 0);

      // RESET STATE
      velocity = VecBuilder.fill(0, 0, 0);
      acceleration = VecBuilder.fill(0,0,0);

      // PREVENTS FRAMES FROM BEING RENDERED
      isBeingLaunched = false;
    }
  }
}
