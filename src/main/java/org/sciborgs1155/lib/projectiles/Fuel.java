package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.Constants.Robot.SHOOTER_LENGTH;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.sciborgs1155.robot.FieldConstants.Hub;

public class Fuel extends Projectile {
  public static final double FUEL_MASS = 0.225;
  public static final double FUEL_RADIUS = 0.075;

  private static final double SCORE_TOLERANCE = 0;
  private static final double GRAVITY = -9.80665;
  private static final double AIR_DENSITY = 1.225;
  private static final double AIR_VISCOSITY = 15.24 * Math.pow(10, -6);

  // MULTIPLY BY VElOCITY SQUARED
  private static final double DRAG_CONSTANT =
    0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS;

  // MULTIPLY BY VELOCITY SQUARED * ANGULAR SPEED
  private static final double LIFT_CONSTANT = 
    (4/3) * 4 * Math.pow(Math.PI,2)* Math.pow(FUEL_RADIUS,3) * AIR_DENSITY;
  
  // MULTIPLY BY ANGULAR SPEED
  private static final double TORQUE_CONSTANT = -8 * Math.PI * AIR_VISCOSITY * Math.pow(FUEL_RADIUS, 3);

  private static double launchSpeed(double wheelVelocity) {
    // SOURCE: https://www.chiefdelphi.com/t/new-flywheel-shooter-analysis/439111/4
    return wheelVelocity / 500; // TODO: Implement.
  }

  private static double[] toLaunchDirectionVector(double pitch, double yaw) {
    return new double[]{Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)};
  }

  private static double[] applyYaw(double[] vector, double yaw) {
    return new double[]{vector[X] * Math.cos(yaw) - vector[Y] * Math.sin(yaw), vector[X] * Math.sin(yaw) + vector[Y] * Math.cos(yaw), vector[Z]};
  }

  private static double norm(double[] vector) {
    return Math.sqrt(vector[X] * vector[X] + vector[Y] * vector[Y] + vector[Z] * vector[Z]);
  }

  private static double sum(double[] vector) {
    return vector[X] + vector[Y] + vector[Z];
  }

  public static double[] shooterVelocity(double[] robotRelativeLaunchDirection, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    // CALCULATE LAUNCH TRANSLATION (ROBOT RELATIVE)
    double shooterLength = SHOOTER_LENGTH.in(Meters);

    double robotRelativeLaunchX = ROBOT_TO_SHOOTER.getX() + robotRelativeLaunchDirection[X] * shooterLength;
    double robotRelativeLaunchY = ROBOT_TO_SHOOTER.getY() + robotRelativeLaunchDirection[Y] * shooterLength;
    double robotRelativeLaunchZ = ROBOT_TO_SHOOTER.getZ() + robotRelativeLaunchDirection[Z] * shooterLength;

    // ADD ROBOT ROTATIONAL VELOCITY
    double rotationalSpeed = robotVelocity.omegaRadiansPerSecond * Math.sqrt(robotRelativeLaunchX * robotRelativeLaunchX + robotRelativeLaunchY * robotRelativeLaunchY + robotRelativeLaunchZ * robotRelativeLaunchZ);
    double centripetalDirection = robotPose.getRotation().getZ();
    double tangentialDirection = centripetalDirection + Math.PI / 2.0;

    double rotationalVelocityX = rotationalSpeed * Math.cos(tangentialDirection);
    double rotationalVelocityY = rotationalSpeed * Math.sin(tangentialDirection);

    // ADD ROBOT TRANSLATIONAL VELOCITY
    double translationVelocityX = robotVelocity.vxMetersPerSecond;
    double translationVelocityY = robotVelocity.vyMetersPerSecond;

    return new double[] {rotationalVelocityX + translationVelocityX, rotationalVelocityY + translationVelocityY, 0};
  }

  public static double[] launchTranslation(double pitch, double yaw, Pose3d robotPose) {
    return launchTranslation(toLaunchDirectionVector(pitch, yaw), robotPose);
  }

  public static double[] launchTranslation(double[] robotRelativeLaunchDirection, Pose3d robotPose) {    
    double[] robotRelativeShooterTranslation = applyYaw(ROBOT_TO_SHOOTER.toVector().getData(), robotPose.getRotation().getZ());
    double[] fieldRelativeShooterTranslation = {robotRelativeShooterTranslation[X] + robotPose.getTranslation().getX(), robotRelativeShooterTranslation[Y] + robotPose.getTranslation().getY(), robotRelativeShooterTranslation[Z] + robotPose.getTranslation().getZ()};
    double[] fieldRelativeLaunchDirection = applyYaw(robotRelativeLaunchDirection, robotPose.getRotation().getZ());

    double shooterLength = SHOOTER_LENGTH.in(Meters);
    return new double[] {
      fieldRelativeShooterTranslation[X] + fieldRelativeLaunchDirection[X] * shooterLength,
      fieldRelativeShooterTranslation[Y] + fieldRelativeLaunchDirection[Y] * shooterLength,
      fieldRelativeShooterTranslation[Z] + fieldRelativeLaunchDirection[Z] * shooterLength
    };
  }

  public static double[] launchVelocity(
      double wheelVelocity,
      double pitch,
      double yaw,
      Pose3d robotPose,
      ChassisSpeeds robotVelocity) {
    double shotSpeed = launchSpeed(wheelVelocity);

    double[] robotRelativeLaunchDirection = toLaunchDirectionVector(pitch, yaw);
    double[] fieldRelativeLaunchDirection = applyYaw(robotRelativeLaunchDirection, robotPose.getRotation().getZ());
    double[] shotVelocity = {fieldRelativeLaunchDirection[X] * shotSpeed, fieldRelativeLaunchDirection[Y] * shotSpeed, fieldRelativeLaunchDirection[Z] * shotSpeed};

    return launchVelocity(shotVelocity, robotPose, robotVelocity);
  }

  public static double[] launchVelocity(double[] fieldRelativeShotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double heading = robotPose.getRotation().getZ();

    double[] robotRelativeLaunchVector = new double[] {fieldRelativeShotVelocity[X] * Math.cos(heading) + fieldRelativeShotVelocity[Y] * Math.sin(heading), -fieldRelativeShotVelocity[X] * Math.sin(heading) + fieldRelativeShotVelocity[Y] * Math.cos(heading), fieldRelativeShotVelocity[Z]};
    double robotRelativeLaunchVectorNorm = norm(robotRelativeLaunchVector);

    if (robotRelativeLaunchVectorNorm == 0.0) return fieldRelativeShotVelocity; // No direction, so no shooter velocity contribution.

    double[] robotRelativeLaunchDirection = {robotRelativeLaunchVector[X] / robotRelativeLaunchVectorNorm, robotRelativeLaunchVector[Y] / robotRelativeLaunchVectorNorm, robotRelativeLaunchVector[Z] / robotRelativeLaunchVectorNorm};
    double[] shooterVelocity = shooterVelocity(robotRelativeLaunchDirection, robotPose, robotVelocity);

    return new double[] {
      fieldRelativeShotVelocity[X] + shooterVelocity[X],
      fieldRelativeShotVelocity[Y] + shooterVelocity[Y],
      fieldRelativeShotVelocity[Z] + shooterVelocity[Z]
    };
  }

  public static double[] launchRotation(Pose3d robotPose) {
    return new double[] {
      robotPose.getRotation().getX(), 
      robotPose.getRotation().getY(), 
      robotPose.getRotation().getZ()
    };
  }

  public static double[] launchRotationalVelocity(Pose3d robotPose) {
    return new double[]{0,0,0}; // TODO: Implement.
  }

  @Override
  protected double[] gravity() {
    // SOURCE: https://spaceplace.nasa.gov/what-is-gravity/en/
    return new double[] {0, 0, GRAVITY};
  }

  @Override
  protected double[] drag() {
    // SOURCE: https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
    return new double[] {
      velocity[X] * velocity[X] * DRAG_CONSTANT,
      velocity[Y] * velocity[Y] * DRAG_CONSTANT,
      velocity[Z] * velocity[Z] * DRAG_CONSTANT
    };
  }

  @Override
  protected double[] torque() {
    // TORQUE CALCULATIONS (METERS / FRAME^2)
    // SOURCE:
    // https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_1998_Grugel.pdf
    return new double[] {rotationalVelocity[X] * TORQUE_CONSTANT, rotationalVelocity[Y] * TORQUE_CONSTANT, rotationalVelocity[Z] * TORQUE_CONSTANT};
  }

  @Override
  protected double[] lift() {
    // SOURCE:
    // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
    return new double[] {0, 0, LIFT_CONSTANT * Math.pow(norm(velocity),2) * sum(rotationalVelocity)};
  }

  @Override
  protected boolean checkIfScored() {
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
  protected boolean checkIfMissed() {
    return translation[Z] <= FUEL_RADIUS;
  }
}
