package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.Constants.Robot.SHOOTER_LENGTH;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.sciborgs1155.robot.FieldConstants.Hub;

/**
 * Models the physics of a fuel projectile (sphere) including translation, drag, lift and torque.
 */
public class Fuel extends Projectile {
  /** Mass of the fuel projectile in kilograms. */
  public static final double FUEL_MASS = 0.225;

  /** Radius of the fuel projectile in meters. */
  public static final double FUEL_RADIUS = 0.075;

  private static final double SCORE_TOLERANCE = 0;
  private static final double GRAVITY = -9.80665;
  private static final double AIR_DENSITY = 1.225;
  private static final double AIR_VISCOSITY = 15.24 * Math.pow(10, -6);

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
   * Converts pitch and yaw angles (radians) to a unit direction vector.
   *
   * @param pitch elevation angle in radians
   * @param yaw heading angle in radians
   * @return 3-element direction vector [x, y, z]
   */
  private static double[] toDirectionVector(double pitch, double yaw) {
    return new double[] {
      Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)
    };
  }

  /**
   * Rotates a 3D vector by a yaw angle about the Z axis.
   *
   * @param vector 3-element vector [x, y, z]
   * @param yaw rotation about Z in radians
   * @return rotated 3-element vector
   */
  private static double[] applyYaw(double[] vector, double yaw) {
    return new double[] {
      vector[X] * Math.cos(yaw) - vector[Y] * Math.sin(yaw),
      vector[X] * Math.sin(yaw) + vector[Y] * Math.cos(yaw),
      vector[Z]
    };
  }

  /**
   * Computes the Euclidean norm of a 3-element vector.
   *
   * @param vector 3-element vector
   * @return length (>= 0)
   */
  private static double norm(double[] vector) {
    return Math.sqrt(vector[X] * vector[X] + vector[Y] * vector[Y] + vector[Z] * vector[Z]);
  }

  /**
   * Computes the linear velocity contribution from the robot to the launched projectile, combining
   * translational and rotational components at the shooter exit point.
   *
   * @param robotRelativeLaunchDirection unit vector in robot-relative launch direction
   * @param robotPose robot pose on the field
   * @param robotVelocity robot chassis speeds
   * @return field-relative velocity contribution [vx, vy, vz]
   */
  protected static double[] shooterVelocity(
      double[] robotRelativeLaunchDirection, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    // CALCULATE LAUNCH TRANSLATION (ROBOT RELATIVE)
    double shooterLength = SHOOTER_LENGTH.in(Meters);

    double robotRelativeLaunchX =
        ROBOT_TO_SHOOTER.getX() + robotRelativeLaunchDirection[X] * shooterLength;
    double robotRelativeLaunchY =
        ROBOT_TO_SHOOTER.getY() + robotRelativeLaunchDirection[Y] * shooterLength;
    double robotRelativeLaunchZ =
        ROBOT_TO_SHOOTER.getZ() + robotRelativeLaunchDirection[Z] * shooterLength;

    // ADD ROBOT ROTATIONAL VELOCITY
    double rotationalSpeed =
        robotVelocity.omegaRadiansPerSecond
            * Math.sqrt(
                robotRelativeLaunchX * robotRelativeLaunchX
                    + robotRelativeLaunchY * robotRelativeLaunchY
                    + robotRelativeLaunchZ * robotRelativeLaunchZ);
    double centripetalDirection = robotPose.getRotation().getZ();
    double tangentialDirection = centripetalDirection + Math.PI / 2.0;

    double rotationalVelocityX = rotationalSpeed * Math.cos(tangentialDirection);
    double rotationalVelocityY = rotationalSpeed * Math.sin(tangentialDirection);

    // ADD ROBOT TRANSLATIONAL VELOCITY
    double translationVelocityX = robotVelocity.vxMetersPerSecond;
    double translationVelocityY = robotVelocity.vyMetersPerSecond;

    return new double[] {
      rotationalVelocityX + translationVelocityX, rotationalVelocityY + translationVelocityY, 0
    };
  }

  /**
   * Computes the field-relative launch position of the projectile given a robot-relative launch
   * direction.
   *
   * @param robotRelativeLaunchDirection robot-relative unit direction vector
   * @param robotPose robot pose on the field
   * @return field-relative 3-element translation [x, y, z] of the launch point
   */
  protected static double[] launchTranslation(
      double[] robotRelativeLaunchDirection, Pose3d robotPose) {
    double[] robotRelativeShooterTranslation =
        applyYaw(ROBOT_TO_SHOOTER.toVector().getData(), robotPose.getRotation().getZ());
    double[] fieldRelativeShooterTranslation = {
      robotRelativeShooterTranslation[X] + robotPose.getTranslation().getX(),
      robotRelativeShooterTranslation[Y] + robotPose.getTranslation().getY(),
      robotRelativeShooterTranslation[Z] + robotPose.getTranslation().getZ()
    };
    double[] fieldRelativeLaunchDirection =
        applyYaw(robotRelativeLaunchDirection, robotPose.getRotation().getZ());

    double shooterLength = SHOOTER_LENGTH.in(Meters);
    return new double[] {
      fieldRelativeShooterTranslation[X] + fieldRelativeLaunchDirection[X] * shooterLength,
      fieldRelativeShooterTranslation[Y] + fieldRelativeLaunchDirection[Y] * shooterLength,
      fieldRelativeShooterTranslation[Z] + fieldRelativeLaunchDirection[Z] * shooterLength
    };
  }

  /**
   * Convenience overload for launchTranslation using pitch and yaw angles.
   *
   * @param pitch launch elevation in radians
   * @param yaw launch heading in radians
   * @param robotPose robot pose on the field
   * @return field-relative launch position [x, y, z]
   */
  protected static double[] launchTranslation(double pitch, double yaw, Pose3d robotPose) {
    return launchTranslation(toDirectionVector(pitch, yaw), robotPose);
  }

  /**
   * Computes the resulting field-relative launch velocity, including robot motion, for a given
   * muzzle speed and launch angles.
   *
   * @param velocity scalar launch speed (m/s)
   * @param pitch elevation angle (radians)
   * @param yaw heading angle (radians)
   * @param robotPose robot pose on the field
   * @param robotVelocity robot chassis speeds
   * @return field-relative 3-element velocity [vx, vy, vz]
   */
  protected static double[] launchVelocity(
      double velocity, double pitch, double yaw, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double[] robotRelativeLaunchDirection = toDirectionVector(pitch, yaw);
    double[] fieldRelativeLaunchDirection =
        applyYaw(robotRelativeLaunchDirection, robotPose.getRotation().getZ());
    double[] shotVelocity = {
      fieldRelativeLaunchDirection[X] * velocity,
      fieldRelativeLaunchDirection[Y] * velocity,
      fieldRelativeLaunchDirection[Z] * velocity
    };

    return launchVelocity(shotVelocity, robotPose, robotVelocity);
  }

  /**
   * Adds robot-induced velocity (translation + rotation at the shooter) to a field-relative shot
   * velocity.
   *
   * @param fieldRelativeShotVelocity shot velocity expressed in field coordinates
   * @param robotPose robot pose on the field
   * @param robotVelocity robot chassis speeds
   * @return combined field-relative velocity [vx, vy, vz]
   */
  @SuppressWarnings("PMD.AvoidLiteralsInIfCondition")
  protected static double[] launchVelocity(
      double[] fieldRelativeShotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double heading = robotPose.getRotation().getZ();

    double[] robotRelativeLaunchVector = {
      fieldRelativeShotVelocity[X] * Math.cos(heading)
          + fieldRelativeShotVelocity[Y] * Math.sin(heading),
      -fieldRelativeShotVelocity[X] * Math.sin(heading)
          + fieldRelativeShotVelocity[Y] * Math.cos(heading),
      fieldRelativeShotVelocity[Z]
    };
    double robotRelativeLaunchVectorNorm = norm(robotRelativeLaunchVector);

    if (robotRelativeLaunchVectorNorm == 0.0)
      return fieldRelativeShotVelocity; // No direction, so no shooter velocity contribution.

    double[] robotRelativeLaunchDirection = {
      robotRelativeLaunchVector[X] / robotRelativeLaunchVectorNorm,
      robotRelativeLaunchVector[Y] / robotRelativeLaunchVectorNorm,
      robotRelativeLaunchVector[Z] / robotRelativeLaunchVectorNorm
    };
    double[] shooterVelocity =
        shooterVelocity(robotRelativeLaunchDirection, robotPose, robotVelocity);

    return new double[] {
      fieldRelativeShotVelocity[X] + shooterVelocity[X],
      fieldRelativeShotVelocity[Y] + shooterVelocity[Y],
      fieldRelativeShotVelocity[Z] + shooterVelocity[Z]
    };
  }

  /**
   * Computes the rotation axis (quaternion-like [w,x,y,z] with w==0) associated with the launch
   * direction given by pitch and yaw.
   *
   * @param pitch elevation angle (radians)
   * @param yaw heading angle (radians)
   * @param robotPose robot pose on the field
   * @return 4-element array representing rotation axis [w, x, y, z]
   */
  protected static double[] launchRotation(double pitch, double yaw, Pose3d robotPose) {
    double[] robotRelativeLaunchDirection = toDirectionVector(pitch, yaw);
    double[] fieldRelativeLaunchDirection =
        applyYaw(robotRelativeLaunchDirection, robotPose.getRotation().getZ());

    return launchRotation(fieldRelativeLaunchDirection, robotPose);
  }

  /**
   * Computes a unit rotation axis perpendicular to the shot direction in field coordinates.
   *
   * @param fieldRelativeShotDirection 3-element shot direction in field coordinates
   * @param robotPose robot pose (unused for current implementation but kept for API symmetry)
   * @return 4-element axis [w, x, y, z] (w is zero)
   */
  @SuppressWarnings("PMD.AvoidLiteralsInIfCondition")
  protected static double[] launchRotation(double[] fieldRelativeShotDirection, Pose3d robotPose) {
    double[] axis = applyYaw(fieldRelativeShotDirection, Math.PI / 2.0);
    double norm = norm(axis);
    if (norm == 0.0) return new double[] {0, 0, 0, 0};

    return new double[] {0, axis[X] / norm, axis[Y] / norm, 0};
  }

  /**
   * Returns the rotational speed (rad/s) imparted to the projectile at launch.
   *
   * <p>TODO: implement a physics-based calculation. Returns a placeholder currently.
   *
   * @return rotational velocity (rad/s)
   */
  protected static double launchRotationalVelocity() {
    return 0.5;
  }

  @Override
  protected double[] weight() {
    // SOURCE: https://spaceplace.nasa.gov/what-is-gravity/en/
    return new double[] {0, 0, GRAVITY};
  }

  @Override
  protected double[] drag() {
    // SOURCE: https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
    return new double[] {
      velocity[X] * velocity[X] * DRAG_CONSTANT / FUEL_MASS,
      velocity[Y] * velocity[Y] * DRAG_CONSTANT / FUEL_MASS,
      velocity[Z] * velocity[Z] * DRAG_CONSTANT / FUEL_MASS
    };
  }

  @Override
  protected double torque() {
    // SOURCE:
    // https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_1998_Grugel.pdf
    return rotationalVelocity * TORQUE_CONSTANT / FUEL_MASS;
  }

  @Override
  protected double[] lift() {
    // SOURCE:
    // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
    return new double[] {0, 0, LIFT_CONSTANT * norm(velocity) * rotationalVelocity / FUEL_MASS};
  }

  @Override
  protected boolean willScore() {
    Translation2d translation2d = new Translation2d(translation[X], translation[Y]);

    double planarDisplacement =
        Math.min(
            translation2d.getDistance(Hub.TOP_CENTER_POINT.toTranslation2d()),
            translation2d.getDistance(Hub.OPP_TOP_CENTER_POINT.toTranslation2d()));
    double verticalDisplacement = Hub.HEIGHT - translation[Z];
    double scoreRadius = SCORE_TOLERANCE + FUEL_RADIUS + Hub.WIDTH / 2;

    return (verticalDisplacement < 0)
        && (verticalDisplacement > -FUEL_RADIUS)
        && (planarDisplacement <= scoreRadius)
        && velocity[Z] < 0;
  }

  @Override
  protected boolean willMiss() {
    return translation[Z] <= FUEL_RADIUS;
  }
}
