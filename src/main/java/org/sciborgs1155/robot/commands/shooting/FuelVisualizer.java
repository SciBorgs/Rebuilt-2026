package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Robot.FLYWHEEL_LIFT;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.Constants.Robot.SHOOTER_TO_FLYWHEEL;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;

/**
 * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
 *
 * @see Fuel
 */
public class FuelVisualizer extends ProjectileVisualizer {
  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
   *
   * @param launchVelocity a supplier that provides the velocity of the FUEL at launch time
   * @param robotPose a supplier that provides the pose of the robot at launch time
   * @param robotVelocity a supplier that provides the velocity of the robot at launch time
   */
  public FuelVisualizer(
      Supplier<double[]> launchVelocity,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> launchTranslation(launchVelocity.get(), robotPose.get()),
        () -> launchVelocity(launchVelocity.get(), robotPose.get(), robotVelocity.get()),
        () -> launchRotation(launchVelocity.get(), robotPose.get()),
        () -> 0);
  }

  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
   *
   * @param launchTranslation a supplier that provides the translation of the FUEL at launch time
   * @param launchVelocity a supplier that provides the velocity of the FUEL at launch time
   * @param launchRotation a supplier that provides the rotation of the FUEL at launch time
   * @param launchRotationalVelocity a supplier that provides the rotational velocity of the FUEL at
   *     launch time
   */
  public FuelVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    super(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);
  }

  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
   *
   * @param launchParameters a supplier for the launch parameters [X, SPEED, PITCH, YAW] of the FUEL
   * @param drive the drivetrain subsystem
   * @return a new visualizer instance
   */
  public static FuelVisualizer fromLaunchParameters(
      Supplier<double[]> launchParameters, Drive drive) {
    return new FuelVisualizer(
        () ->
            launchVelocity(
                Fuel.shotVelocity(launchParameters.get(), drive.pose3d().getRotation().getZ()),
                drive.pose3d(),
                drive.fieldRelativeChassisSpeeds()),
        drive::pose3d,
        drive::fieldRelativeChassisSpeeds);
  }

  @Override
  protected Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled) {
    return new Fuel().config(resolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
  }

  protected static double distanceToHub(double[] shotVelocity, Pose3d robotPose) {
    double[] launchTranslation = launchTranslation(shotVelocity, robotPose);
    double[] shooterToHub = sub3(fromTranslation(Hub.TOP_CENTER_POINT), launchTranslation);

    return Math.hypot(shooterToHub[X], shooterToHub[Y]);
  }

  protected static double[] launchTranslation(double[] shotVelocity, Pose3d robotPose) {
    double[] robotTranslation = {robotPose.getX(), robotPose.getY(), robotPose.getZ()};
    double[] robotToFuel = add3(shooterToFuel(shotVelocity, robotPose), shooterPose(robotPose));

    return add3(robotToFuel, robotTranslation);
  }

  protected static double[] launchVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return add3(shotVelocity, shooterVelocity(shotVelocity, robotPose, robotVelocity));
  }

  protected static double[] launchRotation(double[] shotVelocity, Pose3d robotPose) {
    double[] axis = rotateAroundZ(shotVelocity, Math.PI / 2.0);
    return scale4(new double[] {0, axis[X], axis[Y], axis[Z]}, 1 / norm3(shotVelocity));
  }

  protected static double[] shooterPose(Pose3d robotPose) {
    return rotateAroundZ(fromTranslation(ROBOT_TO_SHOOTER), robotPose.getRotation().getZ());
  }

  protected static double[] shooterVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double[] robotToFuel = add3(shooterToFuel(shotVelocity, robotPose), shooterPose(robotPose));
    double tangentialSpeed = robotVelocity.omegaRadiansPerSecond * norm3(robotToFuel);
    double tangentialDirection = robotPose.getRotation().getZ() + Math.PI / 2.0;

    return new double[] {
      robotVelocity.vxMetersPerSecond + tangentialSpeed * Math.cos(tangentialDirection),
      robotVelocity.vyMetersPerSecond + tangentialSpeed * Math.sin(tangentialDirection),
      0
    };
  }

  protected static double[] shooterToFuel(double[] shotVelocity, Pose3d robotPose) {
    double[] shotDirection = scale3(shotVelocity, 1 / norm3(shotVelocity));

    double shooterToFlyWheel = Math.hypot(shotDirection[X], shotDirection[Y]);
    double angle = Math.atan2(shotDirection[Z], shooterToFlyWheel);

    double[] flywheelToFuel = {
      -Math.cos(angle) * SHOOTER_TO_FLYWHEEL.in(Meters),
      0,
      Math.sin(angle) * SHOOTER_TO_FLYWHEEL.in(Meters)
    };

    double[] shooterToFlywheel = {SHOOTER_TO_FLYWHEEL.in(Meters), 0, FLYWHEEL_LIFT.in(Meters)};
    return rotateAroundZ(add3(shooterToFlywheel, flywheelToFuel), robotPose.getRotation().getZ());
  }

  /** Models the launch physics of a FUEL projectile. */
  public static class Fuel extends Projectile {
    protected static final double FUEL_MASS = 0.225;
    protected static final double FUEL_RADIUS = 0.075;

    protected double scoreTolerance = 0;
    protected double scoreDepth = 0;

    protected static final double GRAVITY = -9.80665;
    protected static final double AIR_DENSITY = 1.225;
    protected static final double AIR_VISCOSITY = 15.24 * Math.pow(10, -6);

    /** Multiplied by velocity squared to compute drag force. */
    private static final double DRAG_CONSTANT =
        0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS;

    /** Multiplied by velocity * angular speed to compute lift force. */
    private static final double LIFT_CONSTANT =
        4 / 3 * 4 * Math.PI * Math.PI * FUEL_RADIUS * FUEL_RADIUS * FUEL_RADIUS * AIR_DENSITY;

    /** Multiplied by angular speed to compute torque. */
    private static final double TORQUE_CONSTANT =
        -8 * Math.PI * AIR_VISCOSITY * Math.pow(FUEL_RADIUS, 3);

    /**
     * Alters scoring parameters.
     * 
     * @param tolerance the maximum planar distance from the HUB
     * @param depth the distance under the top of the HUB.
     * @return this FUEL for chaining
     */
    public Fuel withScoringParameters(double tolerance, double depth) {
      scoreDepth = depth;
      scoreTolerance = tolerance;

      return this;
    }

    @Override
    protected double[] weight() {
      // SOURCE: https://spaceplace.nasa.gov/what-is-gravity/en/
      return new double[] {0, 0, GRAVITY};
    }

    @Override
    protected double[] drag() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
      return new double[] {
        Math.copySign(velocity[X] * velocity[X] * DRAG_CONSTANT / FUEL_MASS, -velocity[X]),
        Math.copySign(velocity[Y] * velocity[Y] * DRAG_CONSTANT / FUEL_MASS, -velocity[Y]),
        Math.copySign(velocity[Z] * velocity[Z] * DRAG_CONSTANT / FUEL_MASS, -velocity[Z])
      };
    }

    @Override
    protected double torque() {
      // https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_1998_Grugel.pdf
      return rotationalVelocity * TORQUE_CONSTANT / FUEL_MASS;
    }

    @Override
    protected double[] lift() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
      return new double[] {0, 0, LIFT_CONSTANT * norm3(velocity) * rotationalVelocity / FUEL_MASS};
    }

    @Override
    protected boolean willScore() {
      double hub1XDisplacement = translation[X] - Hub.TOP_CENTER_POINT.getX();
      double hub1YDisplacement = translation[Y] - Hub.TOP_CENTER_POINT.getY();

      double hub2XDisplacement = translation[X] - Hub.OPP_TOP_CENTER_POINT.getX();
      double hub2YDisplacement = translation[Y] - Hub.OPP_TOP_CENTER_POINT.getY();

      double hub1Distance = Math.hypot(hub1XDisplacement, hub1YDisplacement);
      double hub2Distance = Math.hypot(hub2XDisplacement, hub2YDisplacement);

      double planarDistance = Math.min(hub1Distance, hub2Distance);
      double verticalDisplacement = Hub.HEIGHT - scoreDepth - translation[Z];
      double scoreRadius = scoreTolerance + FUEL_RADIUS + Hub.WIDTH / 2;

      return verticalDisplacement < 0
          && verticalDisplacement > -FUEL_RADIUS
          && planarDistance <= scoreRadius
          && velocity[Z] < 0;
    }

    @Override
    protected boolean willMiss() {
      double hub1XDisplacement = translation[X] - Hub.TOP_CENTER_POINT.getX();
      double hub1YDisplacement = translation[Y] - Hub.TOP_CENTER_POINT.getY();

      double hub2XDisplacement = translation[X] - Hub.OPP_TOP_CENTER_POINT.getX();
      double hub2YDisplacement = translation[Y] - Hub.OPP_TOP_CENTER_POINT.getY();

      double hub1Distance = Math.hypot(hub1XDisplacement, hub1YDisplacement);
      double hub2Distance = Math.hypot(hub2XDisplacement, hub2YDisplacement);

      double planarDistance = Math.min(hub1Distance, hub2Distance);
      double verticalDisplacement = Hub.HEIGHT - scoreDepth - translation[Z];
      double scoreRadius = scoreTolerance + FUEL_RADIUS + Hub.WIDTH / 2;

      return (verticalDisplacement > -FUEL_RADIUS
              && planarDistance > scoreRadius
              && velocity[Z] < 0)
          || translation[Z] < FUEL_RADIUS;
    }

    protected static double[] shotVelocity(double[] launchParameters, double heading) {
      double[] direction = toDirectionVector(launchParameters[PITCH], launchParameters[YAW]);
      return scale3(
          rotateAroundZ(direction, heading), launchParameters[SPEED]);
    }

    protected static double[] launchParameters(double[] shotVelocity, double heading) {
      double speed = norm3(shotVelocity);
      double[] direction = scale3(shotVelocity, 1 / speed);

      double yaw = Math.atan2(direction[Y], direction[X]) - heading;
      double pitch = Math.asin(direction[Z]);

      return new double[] {0, speed, pitch, yaw};
    }
  }
}
