package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.FieldConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.Constants.Robot;

/** Simulates the behavior of multiple Fuel projectiles using {@code FuelSim}. */
public final class FuelVisualizer {
  /** The mass of the Fuel (KILOGRAMS). */
  private static final double FUEL_MASS = 0.225;

  /** Force due to gravity (METERS / SECOND^2). */
  private static final double GRAVITY = -9.81;

  /** The mass density of the air (KILOGRAMS / METER^3). */
  private static final double AIR_DENSITY = 1.225;

  /** The radius of the Fuel (METERS). */
  private static final double FUEL_RADIUS = 0.075;

  /** How long each frame of the animation is (SECONDS / FRAME). */
  private static final double FRAME_LENGTH = 0.02;

  /** How fast the shooter can launch Fuel (SECONDS / FUEL LAUNCH). */
  private static final double COOLDOWN = 0.05;

  /** The radius of the shooter wheel (METERS). */
  private static final double SHOOTER_WHEEL_RADIUS = 0.1016;

  /** The mass of the shooter wheel (KILOGRAMS). */
  private static final double SHOOTER_WHEEL_MASS = 0.27215542;

  /** This constructor is not meant to be used. */
  private FuelVisualizer() {}

  /** The index of the latest Fuel to have been launched. */
  private static int fuelIndex;

  /** When deleting {@code FuelSim}'s , their scores are stored here. */
  private static int deletedScores;

  /** All Fuel currently being simulated. */
  private static List<FuelSim> fuelSims;

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

  /** If True, all Fuel launch data is retrieved from vector suppliers. */
  private static boolean vectorControl;

  /** A supplier for the launch velocity of the Fuel. Only used in Vector control mode. */
  private static Supplier<Vector<N3>> fuelLaunchVelocity;

  /**
   * To be called on robot startup. Parameters used to calculate Fuel trajectory after launch.
   *
   * @param shooterVelocitySupplier A supplier for the angular velocity of the {@code Shooter}.
   * @param turretAngleSupplier A supplier for the angle of the {@code Turret}.
   * @param hoodAngleSupplier A supplier for the angle of the {@code Hood}.
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
   */
  public static void init(
      Supplier<AngularVelocity> shooterVelocitySupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier,
      Supplier<Pose3d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    shooterVelocity = shooterVelocitySupplier;
    turretAngle = turretAngleSupplier;
    hoodAngle = hoodAngleSupplier;
    robotPose = robotPoseSupplier;
    robotVelocity = robotVelocitySupplier;

    // DISABLE VECTOR CONTROL
    fuelLaunchVelocity = () -> VecBuilder.fill(0, 0, 0);
    vectorControl = false;

    // FUEL INSTANTIATION
    fuelSims = new ArrayList<>(1);
    fuelSims.add(new FuelSim());
  }

  /**
   * To be called on robot startup. Parameters used to calculate Fuel trajectory after launch. <br>
   * </br> {@code NOTE: This enables vector control mode.}
   *
   * @param fuelLaunchVelocitySupplier A supplier for the robot-relative launch velocity of the Fuel
   *     (X, Y, Z : METERS PER SECOND)
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
   */
  public static void init(
      Supplier<Vector<N3>> fuelLaunchVelocitySupplier,
      Supplier<Pose3d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    shooterVelocity = RadiansPerSecond::zero;
    turretAngle = Radians::zero;
    hoodAngle = Radians::zero;
    robotPose = robotPoseSupplier;
    robotVelocity = robotVelocitySupplier;

    // ENABLE VECTOR CONTROL
    fuelLaunchVelocity = fuelLaunchVelocitySupplier;
    vectorControl = true;

    // FUEL INSTANTIATION
    fuelSims = new ArrayList<>(1);
    fuelSims.add(new FuelSim());
  }

  /** Publishes the Fuel display data to {@code NetworkTables}. */
  public static void periodic() {
    // UPDATING SIMULATIONS
    Tracer.startTrace("Fuel Visualizer");

    int scores = deletedScores;
    for (int index = 0; index < fuelSims.size(); index++) {
      // INCREMENT FRAME
      fuelSims.get(index).nextFrame();

      // UPDATE SCORE
      scores += fuelSims.get(index).scores;

      // DELETE IDLE FUEL SIMS
      if (fuelSims.get(index).hasBeenLaunched) {
        deletedScores += fuelSims.get(index).scores;
        fuelSims.remove(index);
      }
    }

    Tracer.endTrace();

    // PUBLISHING DATA
    LoggingUtils.log("Fuel Visualizer/Fuel Poses", fuelPoses(), Pose3d.struct);
    LoggingUtils.log("Fuel Visualizer/Shooter Pose", shooterPose(), Pose3d.struct);
    LoggingUtils.log("Fuel Visualizer/Scores", scores);
  }

  /**
   * Launches a single Fuel from the robot.
   *
   * @return A command to launch Fuel.
   */
  public static Command launchFuel() {
    return Commands.deferredProxy(() -> Commands.runOnce(getLaunchableFuel()::init))
        .andThen(Commands.waitSeconds(COOLDOWN))
        .withName("LAUNCH FUEL");
  }

  /**
   * Returns the first unlaunched Fuel. If all Fuel has been launched, create a new Fuel.
   *
   * @return The first launchable Fuel.
   */
  private static FuelSim getLaunchableFuel() {
    // RESET INDEX IF OUT OF BOUNDS
    if (fuelIndex >= fuelSims.size() - 1) fuelIndex = 0;

    // WHETHER OR NOT THE ENTIRE ARRAY IS GOING TO BE ITERATED THROUGH
    boolean fullCycle = fuelIndex == 0;

    // ITERATE THROUGH NON-CYCLED FUEL
    for (fuelIndex++; fuelIndex < fuelSims.size(); fuelIndex++)
      if (!fuelSims.get(fuelIndex).isBeingLaunched) return fuelSims.get(fuelIndex);

    // IF ITERATED THROUGH WHOLE ARRAY, CREATE NEW FUEL
    if (fullCycle) {
      fuelSims.add(new FuelSim());
      return fuelSims.get(fuelIndex);
    }

    // ITERATE THROUGH REST OF THE ELEMENTS
    fuelIndex = 0;
    return getLaunchableFuel();
  }

  /**
   * Returns poses of every Fuel currently being simulated.
   *
   * @return The poses of every Fuel currently being simulated (METERS).
   */
  private static Pose3d[] fuelPoses() {
    Pose3d[] fuelPoses = new Pose3d[fuelSims.size()];
    for (int index = 0; index < fuelPoses.length; index++)
      fuelPoses[index] = fuelSims.get(index).pose3d();
    return fuelPoses;
  }

  /**
   * Returns the current pose of the shooter.
   *
   * @return The pose of the shooter (FIELD RELATIVE METERS).
   */
  private static Pose3d shooterPose() {
    if (vectorControl)
      return new Pose3d(
          robotPose.get().getTranslation().plus(Robot.ROBOT_TO_SHOOTER),
          new Rotation3d()); // TODO: Add vector rotation.
    return robotPose
        .get()
        .transformBy(
            new Transform3d(
                Robot.ROBOT_TO_SHOOTER,
                new Rotation3d(Radians.zero(), hoodAngle.get(), turretAngle.get())));
  }

  /**
   * Generates the launch translation vector for the Fuel based on robot properties.
   *
   * @return A vector representing the translation of the Fuel at launch (METERS).
   */
  private static Vector<N3> calculateFuelLaunchTranslation() {
    if (vectorControl)
      return shooterPose()
          .getTranslation()
          .toVector()
          .plus(fuelLaunchVelocity.get().unit().times(Robot.SHOOTER_LENGTH.in(Meters)));
    return shooterPose()
        .getTranslation()
        .toVector()
        .plus(
            fromSphericalCoords(
                Robot.SHOOTER_LENGTH.in(Meters) + FUEL_RADIUS, shooterPose().getRotation()));
  }

  /**
   * Generates the launch velocity vector for the Fuel based on robot properties.
   *
   * @return A vector representing the velocity of the Fuel at launch (METERS / FRAME).
   */
  private static Vector<N3> calculateFuelLaunchVelocity() {

    Vector<N3> rotationalVelocity =
        fromSphericalCoords(
            robotVelocity.get().omegaRadiansPerSecond * Robot.ROBOT_TO_SHOOTER.getNorm(),
            robotPose.get().getRotation().plus(new Rotation3d(0, 0, Math.PI / 2)));
    Vector<N3> translationalVelocity =
        VecBuilder.fill(
            robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond, 0);

    if (vectorControl)
      return fuelLaunchVelocity
          .get()
          .plus(translationalVelocity)
          .plus(rotationalVelocity)
          .times(FRAME_LENGTH);

    // CONVERT FUEL LAUNCH SPEED (SOURCE BELOW)
    // https://docs.carlmontrobotics.org/jupyter_notebooks/files/FlywheelShooter.html#Equation-1
    // USED "WHEEL VELOCITY WITH SHAPE FACTORS" EQUATION
    double massRatio = FUEL_MASS / SHOOTER_WHEEL_MASS;
    double numerator = SHOOTER_WHEEL_RADIUS * shooterVelocity.get().in(RadiansPerSecond);

    // SPECIFIC TO BALL PROJECTILE AND CYLINDRICAL WHEEL
    double shapeFactor = 7 / 5;

    double launchSpeed = numerator / (2 + shapeFactor * massRatio);

    // CALCULATE FUEL LAUNCH VELOCITY
    Vector<N3> stationaryVelocity = fromSphericalCoords(launchSpeed, shooterPose().getRotation());

    return stationaryVelocity
        .plus(translationalVelocity)
        .plus(rotationalVelocity)
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

    /** Once the Fuel ends it's first launch, this will permanently be True. */
    protected boolean hasBeenLaunched;

    /** How many times this Fuel has been scored. */
    protected int scores;

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
    private Vector<N3> acceleration() {
      // GRAVITY CALCULATIONS (METERS / FRAME^2)
      Vector<N3> gravity = VecBuilder.fill(0, 0, GRAVITY).times(Math.pow(FRAME_LENGTH, 2));

      // DRAG CALCULATIONS (METERS / FRAME^2)
      double speedSquared = Math.pow(velocity.norm(), 2); // MAGNITUDE OF VELOCITY^2
      double dragCoefficient = 0.47; // SPECIFIC TO SPHERICAL OBJECTS
      double referenceArea = Math.PI * Math.pow(FUEL_RADIUS, 2); // CROSS SECTION
      double dragFactor = 0.5 * AIR_DENSITY * speedSquared * dragCoefficient * referenceArea;
      Vector<N3> drag = velocity.unit().times(dragFactor).div(FUEL_MASS).times(-1);

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

    /**
     * Whether or not the Fuel is being scored into the Hub (used to end frame generation). A scored
     * Fuel is defined as Fuel which is directly above the Hub with a downwards velocity into the
     * Hub.
     *
     * @return True if the Fuel is destined to be scored. False if the Fuel is not destined to be
     *     scored / if it is not determinable.
     */
    protected boolean scored() {
      double planarDistance =
          Math.hypot(HUB.getX() - translation.get(0), HUB.getY() - translation.get(1));
      double verticalDisplacement = HUB.getZ() - translation.get(2);

      return (verticalDisplacement < 0)
          && (verticalDisplacement > -FUEL_RADIUS)
          && (planarDistance < FUEL_RADIUS + HUB_DIAMETER / 2)
          && velocity.get(2) < 0;
    }

    /**
     * Whether or not the Fuel is grounded (used to end frame generation).
     *
     * @return True if the Fuel is on/below the ground. False if the Fuel is in the air.
     */
    protected boolean grounded() {
      return translation.get(2) < FUEL_RADIUS;
    }

    /** Starts frame generation. Resets Fuel state. */
    private void init() {
      if (isBeingLaunched) return;

      translation = calculateFuelLaunchTranslation();
      velocity = calculateFuelLaunchVelocity();
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
      // IF THE FUEL IS SCORED IN THE HUB, FRAME GENERATION ENDS
      if (grounded() || scored()) end();
    }

    /** Ends frame generation. Freezes Fuel in place. */
    private void end() {
      // PREVENT CLIPPING INTO THE GROUND
      if (grounded()) translation.set(2, 0, FUEL_RADIUS);

      // VISUAL CUE FOR SCORED FUEL
      if (scored()) {
        translation = HUB.toVector().minus(VecBuilder.fill(0, 0, HUB.getZ()));
        scores++;
      }

      velocity = VecBuilder.fill(0, 0, 0);
      acceleration = VecBuilder.fill(0, 0, 0);

      // PREVENTS FRAMES FROM BEING RENDERED
      isBeingLaunched = false;
      hasBeenLaunched = true;
    }
  }
}
