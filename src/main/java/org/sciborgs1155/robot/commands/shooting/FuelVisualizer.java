package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Robot.FLYWHEEL_LIFT;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.Constants.Robot.SHOOTER_LENGTH;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.FieldConstants.Hub;

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
        () -> launchRotationalVelocity());
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

  @Override
  protected Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled) {
    return new Fuel().config(resolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
  }

  protected static double[] launchTranslation(double[] shotVelocity, Pose3d robotPose) {
    double[] robotTranslation = {robotPose.getX(), robotPose.getY(), robotPose.getZ()};
    return Projectile.add3(robotToFuel(shotVelocity, robotPose), robotTranslation);
  }

  protected static double[] launchVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return Projectile.add3(shotVelocity, shooterVelocity(shotVelocity, robotPose, robotVelocity));
  }

  protected static double[] launchRotation(double[] shotVelocity, Pose3d robotPose) {
    double[] axis = Projectile.rotateAroundZ(shotVelocity, Math.PI / 2.0);
    return Projectile.scale4(
        new double[] {0, axis[Fuel.X], axis[Fuel.Y], axis[Fuel.Z]},
        1 / Projectile.norm3(shotVelocity));
  }

  protected static double launchRotationalVelocity() {
    return 0.5; // TODO: UPDATE.
  }

  /**
   * Converts shooter properties to a shot velocity vector (X, Y, and Z) which is compatible with
   * visualizers.
   *
   * @param speed the launch speed of the FUEL.
   * @param pitch the pitch of the shooter.
   * @param yaw the yaw of the shooter.
   * @param robotPose the pose of the drivetrain.
   * @return A double[] that can be passed into the constructor of a visualizer.
   */
  public static double[] shotVelocity(double speed, double pitch, double yaw, Pose3d robotPose) {
    return Projectile.scale3(
        Fuel.rotateAroundZ(
            Projectile.toDirectionVector(pitch, yaw), robotPose.getRotation().getZ()),
        speed);
  }

  /**
   * Converts a shooting algorithm output to a shot velocity vector (X, Y, and Z) which is
   * compatible with visualizers.
   *
   * @param shootingAlgorithm the shooting algorithm used to calculate the shot velocity.
   * @param robotPose the pose of the drivetrain.
   * @param robotVelocity the velocity of the drivetrain.
   * @return A double[] that can be passed into the constructor of a visualizer.
   */
  public static double[] shotVelocity(
      ShootingAlgorithm shootingAlgorithm, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return shootingAlgorithm
        .calculate(
            robotPose.getTranslation(),
            VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond))
        .getData();
  }

  protected static double[] robotToFuel(double[] shotVelocity, Pose3d robotPose) {
    double angle =
        Math.atan2(shotVelocity[Fuel.Z], Math.hypot(shotVelocity[Fuel.X], shotVelocity[Fuel.Y]));
    double[] flywheelToFuel = {
      -Math.cos(angle) * SHOOTER_LENGTH.in(Meters), 0, Math.sin(angle) * SHOOTER_LENGTH.in(Meters)
    };
    double[] shooterToFlywheel = {SHOOTER_LENGTH.in(Meters), 0, FLYWHEEL_LIFT.in(Meters)};

    double[] robotToShooter =
        Projectile.rotateAroundZ(
            Projectile.fromTranslation(ROBOT_TO_SHOOTER), robotPose.getRotation().getZ());
    double[] shooterToFuel =
        Projectile.rotateAroundZ(
            Projectile.add3(shooterToFlywheel, flywheelToFuel), robotPose.getRotation().getZ());

    return Projectile.add3(shooterToFuel, robotToShooter);
  }

  protected static double[] shooterVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double tangentialSpeed =
        robotVelocity.omegaRadiansPerSecond
            * Projectile.norm3(robotToFuel(shotVelocity, robotPose));
    double tangentialDirection = robotPose.getRotation().getZ() + Math.PI / 2.0;

    double xVelocity =
        robotVelocity.vxMetersPerSecond + tangentialSpeed * Math.cos(tangentialDirection);
    double yVelocity =
        robotVelocity.vyMetersPerSecond + tangentialSpeed * Math.sin(tangentialDirection);

    return new double[] {xVelocity, yVelocity, 0};
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
  }
}
