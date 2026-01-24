package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
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
  /** This constructor is not meant to be used. */
  private FuelVisualizer() {}

  /** All Fuel currently being simulated. */
  private static FuelSim[] fuelSims;

  /** A supplier for the launch velocity of the Fuel. */
  private static Supplier<AngularVelocity> shooterVelocity;

  /** A supplier for the angle of the turret. */
  private static Supplier<Angle> turretAngleSupplier;

  /** A supplier for the angle of the hood. */
  private static Supplier<Angle> hoodAngleSupplier;

  /** A supplier for the current pose of the robot. */
  private static Supplier<Pose3d> robotPoseSupplier;

  /** The length of each frame in the Fuel launch animation (SECONDS / FRAME). */
  private static final double FRAME_LENGTH = 0.016;

  /** Acceleration due to gravity (METERS / FRAME^2). */
  private static final Vector<N3> GRAVITY = VecBuilder.fill(0, 0, -9.81 * FRAME_LENGTH);

  /** The dimensionless constant multiplied by velocity to attain the drag force. */
  private static final double DRAG_CONSTANT = 0.1;

  /** The mass of the Fuel (KILOGRAMS). */
  private static final double FUEL_MASS = 0.225;

  /** The diameter of the Fuel (METERS). */
  private static final double FUEL_DIAMETER = 0.15;

  /** How fast the Fuel spins in the air (RADIANS / FRAME). Purely cosmetic. */
  private static final double SPIN = 1 * FRAME_LENGTH;

  /**
   * The ratio between the angular velocity of the shooter (RADIANS / SECOND) and the launch
   * velocity of the Fuel (METERS / SECOND).
   */
  private static final double SHOOTER_TO_LAUNCH = 1;

  /**
   * The distance between the origin of the shooter and the actual launch position of the Fuel
   * (METERS).
   */
  private static final double SHOOTER_TO_FUEL = 0;

  /** Once added to the pose of the robot, returns the pose of the shooter (METERS). */
  private static final Vector<N3> ROBOT_TO_SHOOTER = VecBuilder.fill(0, 0, 0.5);

  /**
   * A publisher for the positions of the {@code FuelSim}'s. Used to view Fuel in logging framework.
   */
  private static final StructArrayPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Fuel Visualizer", Pose3d.struct)
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
    FuelVisualizer.shooterVelocity = shooterVelocitySupplier;
    FuelVisualizer.turretAngleSupplier = turretAngleSupplier;
    FuelVisualizer.hoodAngleSupplier = hoodAngleSupplier;
    FuelVisualizer.robotPoseSupplier = robotPoseSupplier;

    // FUEL INSTANTIATION
    FuelVisualizer.fuelSims = new FuelSim[fuelCapacity];
    for (int index = 0; index < fuelSims.length; index++) fuelSims[index] = new FuelSim();
  }

  /**
   * Launches the first idle Fuel in the visualizer. If no idle Fuel are present, the earliest
   * launched Fuel is used.
   *
   * @return A command to launch Fuel.
   */
  public static Command launchFuel() {
    return Commands.runOnce(getNextIdleFuel()::init);
  }

  /** Publishes the Fuel display data to {@code NetworkTables}. */
  public static void periodic() {
    // UPDATING SIMULATIONS
    for (FuelSim fuelSim : fuelSims) {
      if (!fuelSim.isIdle) fuelSim.nextFrame();
      if (fuelSim.hasLanded()) fuelSim.end();
    }

    // LOGGING
    Pose3d[] poses = new Pose3d[fuelSims.length];
    for (int index = 0; index < fuelSims.length; index++)
      if (!fuelSims[index].isIdle) poses[index] = fuelSims[index].pose();
    publisher.accept(poses);
  }

  /**
   * Returns the first idle {@code FuelSim}. If no idle Fuel are present, the earliest launched Fuel
   * is used.
   *
   * @return The first idle Fuel.
   */
  private static FuelSim getNextIdleFuel() {
    for (FuelSim fuelSim : fuelSims) if (fuelSim.isIdle) return fuelSim;
    return fuelSims[0];
  }

  /** Generates the launch translation vector for the Fuel based on robot properties (METERS). */
  private static Vector<N3> getFuelStartingTranslation() {
    Vector<N3> shooterOrigin =
        robotPoseSupplier.get().getTranslation().toVector().plus(ROBOT_TO_SHOOTER);
    Vector<N3> shooterToFuel =
        FieldConstants.fromSphericalCoords(
            SHOOTER_TO_FUEL + FUEL_DIAMETER,
            turretAngleSupplier.get().in(Radians),
            hoodAngleSupplier.get().in(Radians));

    return shooterOrigin.plus(shooterToFuel);
  }

  /**
   * Generates the launch velocity vector for the Fuel based on robot properties (METERS / SECOND).
   */
  private static Vector<N3> getFuelStartingVelocity() {
    return FieldConstants.fromSphericalCoords(
        shooterVelocity.get().in(RadiansPerSecond) * SHOOTER_TO_LAUNCH,
        turretAngleSupplier.get().in(Radians),
        hoodAngleSupplier.get().in(Radians));
  }

  /**
   * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched from
   * the robot using the {@code shoot} command. This Fuel is reused upon the next calling of the
   * command. All units are SI unless specified otherwise.
   */
  public static class FuelSim {
    /**
     * If the Fuel is currently being launched, it is not idle. If the Fuel is at rest, it is idle.
     */
    protected boolean isIdle = true;

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
     * Returns a pose object that contains information about the {@code translation} and {@code
     * rotation} of the Fuel.
     *
     * @return The pose of the Fuel (METERS).
     */
    protected Pose3d pose() {
      return new Pose3d(new Translation3d(translation), new Rotation3d(rotation));
    }

    /**
     * Determines if the Fuel has landed.
     *
     * @param translation The translation of the Fuel.
     * @return True if the translation is above the field. False if it is clipping into the field.
     */
    private boolean hasLanded() {
      return translation.get(2) < FUEL_DIAMETER / 2;
    }

    /** Resets Fuel properties in preparation for launch. */
    private void init() {
      translation = getFuelStartingTranslation();
      velocity = getFuelStartingVelocity().times(FRAME_LENGTH);
      rotation = VecBuilder.fill(0, 0, 0);

      isIdle = false;
    }

    /** Displays the next {@code state} in the {@code trajectory}. To be called periodically. */
    private void nextFrame() {
      velocity.setColumn(0, velocity.plus(GRAVITY));

      // DRAG FORCE IS EQUAL TO NEGATION OF VELOCITY TIMES CONSTANT
      velocity.setColumn(0, velocity.plus(velocity.times(-DRAG_CONSTANT / FUEL_MASS)));

      translation.setColumn(0, translation.plus(velocity));

      // SPIN IS PURELY COSMETIC
      rotation.setColumn(0, rotation.plus(SPIN));
    }

    /** Ends trajectory generation. */
    private void end() {
      // PREVENT CLIPPING INTO FLOOR
      translation.set(2, 0, FUEL_DIAMETER / 2);
      velocity = velocity.times(0);

      isIdle = true;
    }
  }
}
