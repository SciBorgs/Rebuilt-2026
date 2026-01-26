package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.FieldConstants;

/** Simulates the behavior of multiple Fuel projectiles using {@code FuelSim}. */
public final class FuelVisualizer {
  /** The mass of the Fuel (KILOGRAMS). */
  private static final double FUEL_MASS = 0.225;

  /** Force due to gravity (METERS / SECOND^2). */
  private static final double GRAVITY = -9.81;

  /** The mass density of the air (KILOGRAMS / LITER). */
  private static final double AIR_DENSITY = 1.225;

  /** The diameter of the Fuel (METERS). */
  private static final double FUEL_DIAMETER = 0.15;

  /** How long each frame of the animation is (SECONDS / FRAME). */
  private static final double FRAME_LENGTH = 0.02;

  /** This constructor is not meant to be used. */
  private FuelVisualizer() {}

  /** The index of the latest Fuel to have been launched. */
  private static int fuelIndex;

  /** All Fuel currently being simulated. */
  private static FuelSim[] fuelSims;

  /** The poses of every Fuel currently being simulated. */
  private static Pose3d[] fuelPoses;

  /** A supplier for the angular velocity of the shooter. */
  private static Supplier<AngularVelocity> shooterVelocity;

  /** A supplier for the angle of the turret. */
  private static Supplier<Angle> turretAngle;

  /** A supplier for the angle of the hood. */
  private static Supplier<Angle> hoodAngle;

  /** A supplier for the current pose of the robot. */
  private static Supplier<Pose3d> robotPose;

  /** A supplier for the current velocity of the robot. */
  private static Supplier<ChassisSpeeds> robotVelocity;

  /**
   * A publisher for the positions of the {@code FuelSim}'s. Used to track Fuel in logging
   * framework.
   */
  private static StructArrayPublisher<Pose3d> fuelPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Fuel Visualizer/Fuel Poses", Pose3d.struct)
          .publish();

  /**
   * To be called on robot startup. Parameters used to calculate Fuel trajectory after launch.
   *
   * @param shooterVelocitySupplier A supplier for the angular velocity of the {@code Shooter}.
   * @param turretAngleSupplier A supplier for the angle of the {@code Turret}.
   * @param hoodAngleSupplier A supplier for the angle of the {@code Hood}.
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
   * @param fuelCapacity The number of Fuel's to simulate.
   */
  public static void init(
      Supplier<AngularVelocity> shooterVelocitySupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier,
      Supplier<Pose3d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier,
      int fuelCapacity) {
    shooterVelocity = shooterVelocitySupplier;
    turretAngle = turretAngleSupplier;
    hoodAngle = hoodAngleSupplier;
    robotPose = robotPoseSupplier;
    robotVelocity = robotVelocitySupplier;

    // FUEL INSTANTIATION
    fuelSims = new FuelSim[fuelCapacity];
    fuelPoses = new Pose3d[fuelCapacity];

    for (int index = 0; index < fuelSims.length; index++) {
      fuelSims[index] = new FuelSim();
      fuelPoses[index] = new Pose3d();
    }
  }

  /** Publishes the Fuel display data to {@code NetworkTables}. */
  public static void periodic() {
    // UPDATING SIMULATIONS
    Tracer.startTrace("Fuel Visualizer");

    for (int index = 0; index < fuelPoses.length; index++) {
      fuelSims[index].nextFrame();
      fuelPoses[index] = fuelSims[index].pose3d();
    }

    Tracer.endTrace();

    // PUBLISHING DATA
    fuelPosePublisher.accept(fuelPoses);
  }

  /**
   * Launches a single Fuel from the robot.
   *
   * @return A command to launch Fuel.
   */
  public static Command launchFuel() {
    return Commands.deferredProxy(() -> Commands.runOnce(getLaunchableFuel()::init))
        .withName("LAUNCH FUEL");
  }

  /**
   * Returns the first unlaunched Fuel. If all Fuel has been launched, return the first idle Fuel.
   * If no Fuel meets this criteria, return the first Fuel in the visualizer.
   *
   * @return The first launchable Fuel.
   */
  private static FuelSim getLaunchableFuel() {
    // RESET INDEX IF OUT OF BOUNDS
    if (fuelIndex >= fuelSims.length - 1) fuelIndex = 0;

    // WHETHER OR NOT THE ENTIRE ARRAY IS GOING TO BE ITERATED THROUGH
    boolean fullCycle = fuelIndex == 0;

    // ITERATE THROUGH NON-CYCLED FUEL
    for (fuelIndex++; fuelIndex < fuelSims.length; fuelIndex++)
      if (!fuelSims[fuelIndex].isBeingLaunched) return fuelSims[fuelIndex];

    // IF ITERATED THROUGH WHOLE ARRAY, RETURN FIRST FUEL
    if (fullCycle) return fuelSims[0];

    // ITERATE THROUGH REST OF THE ELEMENTS
    fuelIndex = 0;
    return getLaunchableFuel();
  }

  /**
   * Generates the launch translation vector for the Fuel based on robot properties.
   *
   * @return A vector representing the translation of the Fuel at launch (METERS).
   */
  // TODO: ACCOUNT FOR LENGTH OF SHOOTER
  private static Vector<N3> getFuelLaunchTranslation() {
    Vector<N3> shooterOrigin =
        robotPose.get().getTranslation().toVector().plus(VecBuilder.fill(0, 0, 0.5));
    Vector<N3> shooterToFuel =
        FieldConstants.fromSphericalCoords(
            0.1 + FUEL_DIAMETER,
            turretAngle.get().plus(robotPose.get().getRotation().getMeasureZ()).in(Radians),
            hoodAngle.get().in(Radians));

    return shooterOrigin.plus(shooterToFuel);
  }

  /**
   * Generates the launch velocity vector for the Fuel based on robot properties.
   *
   * @return A vector representing the velocity of the Fuel at launch (METERS / FRAME).
   */
  // TODO: ACCOUNT FOR CONVERSION BETWEEN SHOOTER VELOCITY AND LAUNCH VELOCITY
  private static Vector<N3> getFuelLaunchVelocity() {
    return FieldConstants.fromSphericalCoords(
            shooterVelocity.get().in(RadiansPerSecond),
            turretAngle.get().plus(robotPose.get().getRotation().getMeasureZ()).in(Radians),
            hoodAngle.get().in(Radians))
        .plus(
            VecBuilder.fill(
                robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond, 0))
        .times(FRAME_LENGTH);
  }

  /**
   * Simulates a singular {@code Fuel} projectile as it is being launched. The Fuel is launched from
   * the robot using the {@code shoot} command. This Fuel is reused upon the next calling of the
   * command. All units are SI unless specified otherwise.
   */
  protected static class FuelSim {
    /** If the Fuel is suspended in the air, it is being launched. */
    protected boolean isBeingLaunched;

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's
     * translation (METERS).
     */
    protected Vector<N3> translation = VecBuilder.fill(0, 0, 0);

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's velocity
     * (METERS / FRAME).
     */
    protected Vector<N3> velocity = VecBuilder.fill(0, 0, 0);

    /**
     * A vector whose elements represent the current X, Y, and Z components of the Fuel's
     * acceleration (METERS / FRAME^2).
     */
    protected Vector<N3> acceleration = VecBuilder.fill(0, 0, 0);

    /**
     * Generates the acceleration vector for the Fuel based on gravity and drag.
     *
     * @return A vector representing the acceleration of the Fuel (METERS / FRAME^2).
     */
    // TODO: ACCOUNT FOR MAGNUS LIFT.
    private Vector<N3> acceleration() {
      // GRAVITY CALCULATIONS
      Vector<N3> gravity = VecBuilder.fill(0, 0, GRAVITY).times(Math.pow(FRAME_LENGTH, 2));

      // DRAG CALCULATIONS
      double speedSquared = Math.pow(velocity.norm(), 2); // MAGNITUDE OF VELOCITY
      double referenceArea = Math.pow(FUEL_DIAMETER, 2) / 2; // AREA OF SPHERE
      double dragCoefficient = 0.47; // SPECIFIC TO SPHERICAL OBJECTS
      double dragFactor = 0.5 * AIR_DENSITY * speedSquared * dragCoefficient * referenceArea;
      Vector<N3> drag = velocity.unit().times(dragFactor).div(FUEL_MASS);

      return gravity.plus(drag);
    }

    /**
     * Converts {@code translation} and {@code rotation} vectors of the Fuel into a pose object.
     *
     * @return The pose of the Fuel (METERS).
     */
    protected Pose3d pose3d() { // TODO: INCLUDE ROTATION
      return new Pose3d(new Translation3d(translation), new Rotation3d());
    }

    /** Starts frame generation. Resets Fuel state. */
    private void init() {
      if (isBeingLaunched) return;

      translation = getFuelLaunchTranslation();
      velocity = getFuelLaunchVelocity();
      acceleration = acceleration();

      // ALLOWS FRAMES TO BE RENDERED
      isBeingLaunched = true;
    }

    /** Generates a new frame based on Fuel's trajectory. To be called periodically. */
    private void nextFrame() {
      if (!isBeingLaunched) return;

      acceleration.setColumn(0, acceleration());
      velocity.setColumn(0, velocity.plus(acceleration));
      translation.setColumn(0, translation.plus(velocity));

      // IF THE FUEL TOUCHES THE GROUND, FRAME GENERATION ENDS
      if (translation.get(2) < FUEL_DIAMETER / 2) end();
    }

    /** Ends frame generation. Freezes Fuel in place. */
    private void end() {
      // PREVENT CLIPPING INTO THE GROUND
      translation.set(2, 0, FUEL_DIAMETER / 2);
      velocity = VecBuilder.fill(0, 0, 0);
      acceleration = VecBuilder.fill(0, 0, 0);

      // PREVENTS FRAMES FROM BEING RENDERED
      isBeingLaunched = false;
    }
  }
}
