package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.Constants.Robot.SHOOTER_LENGTH;
import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;
import static org.sciborgs1155.robot.FieldConstants.FUEL_RADIUS;
import static org.sciborgs1155.robot.FieldConstants.HUB_DIAMETER;
import static org.sciborgs1155.robot.FieldConstants.HUB_HEIGHT;
import static org.sciborgs1155.robot.FieldConstants.RED_HUB;
import static org.sciborgs1155.robot.FieldConstants.fromPolarCoords;
import static org.sciborgs1155.robot.FieldConstants.fromSphericalCoords;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Fuel extends Projectile {
  public static final double DRAG_CONSTANT =
      0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS;

  private static double launchSpeed(double wheelVelocity) {
    // SOURCE: https://www.chiefdelphi.com/t/new-flywheel-shooter-analysis/439111/4
    return wheelVelocity / 500; // TODO: Implement.
  }

  private static Rotation3d launchDirection(double pitch, double yaw, Pose3d robotPose) {
    return new Rotation3d(0, pitch, yaw).rotateBy(robotPose.getRotation());
  }

  private static Rotation3d launchDirection(double[] directionVector, Pose3d robotPose) {
    return new Rotation3d(
            VecBuilder.fill(directionVector[X], directionVector[Y], directionVector[Z]))
        .rotateBy(robotPose.getRotation());
  }

  protected static double[] launchTranslation(double pitch, double yaw, Pose3d robotPose) {
    return ROBOT_TO_SHOOTER
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation())
        .toVector()
        .plus(
            fromSphericalCoords(SHOOTER_LENGTH.in(Meters), launchDirection(pitch, yaw, robotPose)))
        .getData();
  }

  public static double[] launchTranslation(double[] directionVector, Pose3d robotPose) {
    return ROBOT_TO_SHOOTER
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation())
        .toVector()
        .plus(
            fromSphericalCoords(
                SHOOTER_LENGTH.in(Meters), launchDirection(directionVector, robotPose)))
        .getData();
  }

  protected static double[] launchVelocity(
      double wheelVelocity,
      double pitch,
      double yaw,
      Pose3d robotPose,
      ChassisSpeeds robotVelocity) {
    double[] shotVelocity =
        fromSphericalCoords(launchSpeed(wheelVelocity), launchDirection(pitch, yaw, robotPose))
            .getData();

    double[] shooterVelocity =
        fromPolarCoords(
                robotVelocity.omegaRadiansPerSecond
                    * ROBOT_TO_SHOOTER
                        .toVector()
                        .plus(
                            fromSphericalCoords(
                                SHOOTER_LENGTH.in(Meters), launchDirection(pitch, yaw, robotPose)))
                        .norm(),
                robotPose.toPose2d().getRotation().plus(Rotation2d.kCCW_90deg))
            .plus(VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond))
            .getData();

    return new double[] {
      shotVelocity[X] + shooterVelocity[X], shotVelocity[Y] + shooterVelocity[Y], shotVelocity[Z]
    };
  }

  protected static double[] launchVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double[] shooterVelocity =
        fromPolarCoords(
                robotVelocity.omegaRadiansPerSecond
                    * ROBOT_TO_SHOOTER
                        .toVector()
                        .plus(
                            fromSphericalCoords(
                                SHOOTER_LENGTH.in(Meters),
                                launchDirection(shotVelocity, robotPose)))
                        .norm(),
                robotPose.toPose2d().getRotation().plus(Rotation2d.kCCW_90deg))
            .plus(VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond))
            .getData();

    return new double[] {
      shotVelocity[X] + shooterVelocity[X], shotVelocity[Y] + shooterVelocity[Y], shotVelocity[Z]
    };
  }

  protected static double[] launchRotation(Pose3d robotPose) {
    return new double[] {
      robotPose.getRotation().getX(), robotPose.getRotation().getY(), robotPose.getRotation().getZ()
    };
  }

  protected static double[] launchRotationalVelocity(Pose3d robotPose) {
    return new double[3]; // TODO: Implement.
  }

  @Override
  protected double[] gravity() {
    // SOURCE: https://spaceplace.nasa.gov/what-is-gravity/en/
    return new double[] {0, 0, -9.80665};
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
  protected double[] torque() { // TODO: Implement.
    // TORQUE CALCULATIONS (METERS / FRAME^2)
    // SOURCE:
    // https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_1998_Grugel.pdf
    return new double[] {0, 0, 0};
  }

  @Override
  protected double[] lift() { // TODO: Implement.
    // SOURCE:
    // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
    return new double[] {0, 0, 0};
  }

  @Override
  protected boolean checkIfScored() {
    double distanceFromBlueHub =
        Math.hypot(translation[X] - BLUE_HUB.getX(), translation[Y] - BLUE_HUB.getY());
    double distanceFromRedHub =
        Math.hypot(translation[X] - RED_HUB.getX(), translation[Y] - RED_HUB.getY());

    // FUEL MUST BE DIRECTLY ABOVE THE HUB
    double verticalDisplacement = HUB_HEIGHT.in(Meters) - translation[Z];

    // FUEL MUST BE CLOSE TO THE HUB ON THE 2D PLANE
    double scoreRadius = (FUEL_RADIUS + HUB_DIAMETER.in(Meters) / 2) + 0.5;

    return (verticalDisplacement < 0)
        && (verticalDisplacement > -FUEL_RADIUS)
        && (distanceFromBlueHub < scoreRadius || distanceFromRedHub < scoreRadius)
        // FUEL MUST HAVE A DOWNWARD VELOCITY
        && velocity[Z] < 0;
  }

  @Override
  protected boolean checkIfMissed() {
    return translation[Z] <= FUEL_RADIUS;
  }
}
