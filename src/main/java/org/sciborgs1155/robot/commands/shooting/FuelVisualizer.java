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
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;

/**
 * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
 *
 * @see Fuel
 */
public class FuelVisualizer extends ProjectileVisualizer {
  private double scoreTolerance = Hub.INNER_WIDTH / 2;
  private double scoreDepth;
  private double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);

  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
   *
   * @param launchVelocity a supplier that provides the velocity of the FUEL at launch time
   * @param robotPose a supplier that provides the pose of the robot at launch time
   * @param robotVelocity a supplier that provides the velocity of the robot at launch time
   */
  protected FuelVisualizer(
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
  protected FuelVisualizer(
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

  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
   *
   * @param speed a supplier for the speed of the FUEL at launch
   * @param pitch a supplier for the pitch of the FUEL's launch trajectory
   * @param yaw a supplier for the yaw of the FUEL's launch trajectory
   * @param drive the drivetrain subsystem
   * @return a new visualizer instance
   */
  public static FuelVisualizer fromLaunchParameters(
      DoubleSupplier speed, DoubleSupplier pitch, DoubleSupplier yaw, Drive drive) {
    return new FuelVisualizer(
        () ->
            launchVelocity(
                Fuel.shotVelocity(
                    new double[] {0, speed.getAsDouble(), pitch.getAsDouble(), yaw.getAsDouble()},
                    drive.pose3d().getRotation().getZ()),
                drive.pose3d(),
                drive.fieldRelativeChassisSpeeds()),
        drive::pose3d,
        drive::fieldRelativeChassisSpeeds);
  }

  /**
   * Alters scoring parameters.
   *
   * @param tolerance the maximum planar distance from the HUB
   * @param depth the distance under the top of the HUB.
   * @return this FUEL for chaining
   */
  public FuelVisualizer withScoringParameters(double[] goal, double tolerance, double depth) {
    targetPose = goal.clone();
    scoreDepth = depth;
    scoreTolerance = tolerance;

    return this;
  }

  @Override
  protected Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled) {
    return new Fuel()
        .withScoringParameters(targetPose, scoreTolerance, scoreDepth)
        .config(resolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
  }

  @Override
  public void updateLogging() {
    super.updateLogging();

    if (ending() == null) return;
    double[] planarDistance = {
      ShotGenerator.GOAL[X] - ending().getTranslation().getX(),
      0,
      ShotGenerator.GOAL[Y] - ending().getTranslation().getY()
    };
    LoggingUtils.log("Projectile Visualizer/Distance From Goal", norm3(planarDistance));
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

    protected double scoreDepth;
    protected double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);

    protected double planarDistance = 0;
    protected double verticalDisplacement = 0;
    protected double scoreRadius = Hub.INNER_WIDTH + FUEL_RADIUS;

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
    public Fuel withScoringParameters(double[] goal, double tolerance, double depth) {
      targetPose = goal.clone();
      scoreDepth = depth;
      scoreRadius = tolerance + FUEL_RADIUS;

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
    protected void periodic() {
      super.periodic();

      double xDisplacement = translation[X] - targetPose[X];
      double yDisplacement = translation[Y] - targetPose[Y];

      planarDistance = Math.hypot(xDisplacement, yDisplacement);
      verticalDisplacement = targetPose[Z] - scoreDepth - translation[Z];
    }

    @Override
    protected boolean willScore() {
      return verticalDisplacement < 0
          && verticalDisplacement > -FUEL_RADIUS
          && planarDistance <= scoreRadius
          && velocity[Z] < 0;
    }

    @Override
    protected boolean willMiss() {
      return (verticalDisplacement > -FUEL_RADIUS
              && planarDistance > scoreRadius
              && velocity[Z] < 0)
          || translation[Z] < FUEL_RADIUS;
    }

    protected static double[] shotVelocity(double[] launchParameters, double heading) {
      double[] direction = toDirectionVector(launchParameters[PITCH], launchParameters[YAW]);
      return scale3(rotateAroundZ(direction, heading), launchParameters[SPEED]);
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
