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
  private static final double AIR_DENSITY = 1.225;

  private static final double DRAG_CONSTANT =
      0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS;

  private static double launchSpeed(double wheelVelocity) {
    // SOURCE: https://www.chiefdelphi.com/t/new-flywheel-shooter-analysis/439111/4
    return wheelVelocity / 500; // TODO: Implement.
  }

  // Convert spherical angles (pitch, yaw) in robot frame -> unit direction (robot frame)
  private static void sphericalUnitVectorRobot(double pitch, double yaw, double[] out) {
    double cosP = Math.cos(pitch);
    out[X] = cosP * Math.cos(yaw);
    out[Y] = cosP * Math.sin(yaw);
    out[Z] = Math.sin(pitch);
  }

  // Rotate a 3D vector about the Z axis (yaw). Input and output can be same array.
  private static void rotateAboutZ(double[] in, double yaw, double[] out) {
    double c = Math.cos(yaw);
    double s = Math.sin(yaw);
    double ix = in[X];
    double iy = in[Y];
    out[X] = ix * c - iy * s;
    out[Y] = ix * s + iy * c;
    out[Z] = in[Z];
  }

  // Hot path: compute shooter velocity using primitive vectors (launch direction in robot frame).
  // This avoids creating VecBuilder/Rotation2d objects every call.
  private static double[] shooterVelocity(
      double[] launchDirRobot, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    // Robot->shooter mounting point in robot frame
    double rtx = ROBOT_TO_SHOOTER.getX();
    double rty = ROBOT_TO_SHOOTER.getY();
    double rtz = ROBOT_TO_SHOOTER.getZ();

    // Add shooter length along launch direction (robot frame)
    double shootLen = SHOOTER_LENGTH.in(Meters);
    double tipX = rtx + launchDirRobot[X] * shootLen;
    double tipY = rty + launchDirRobot[Y] * shootLen;
    double tipZ = rtz + launchDirRobot[Z] * shootLen;

    // Linear speed contribution from robot rotation (omega * radius)
    double rotationalSpeed =
        robotVelocity.omegaRadiansPerSecond * Math.sqrt(tipX * tipX + tipY * tipY + tipZ * tipZ);

    // Convert rotational contribution into a planar vector using robot heading + 90deg
    double heading = robotPose.getRotation().getZ(); // yaw (radians)
    double rotHeading = heading + Math.PI / 2.0;
    double rotVx = rotationalSpeed * Math.cos(rotHeading);
    double rotVy = rotationalSpeed * Math.sin(rotHeading);

    // Add chassis linear velocities (vx, vy)
    double chassisVx = robotVelocity.vxMetersPerSecond;
    double chassisVy = robotVelocity.vyMetersPerSecond;

    return new double[] {rotVx + chassisVx, rotVy + chassisVy, 0};
  }

  protected static double[] launchTranslation(double pitch, double yaw, Pose3d robotPose) {
    // Build unit launch direction in robot frame, rotate into field frame and scale by shooter
    // length
    double[] launchDirRobot = new double[3];
    sphericalUnitVectorRobot(pitch, yaw, launchDirRobot);
    return launchTranslation(launchDirRobot, robotPose);
  }

  public static double[] launchTranslation(double[] directionVectorRobotFrame, Pose3d robotPose) {
    // directionVectorRobotFrame is a primitive unit-direction expressed in robot frame.
    var baseTranslation =
        ROBOT_TO_SHOOTER
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation())
            .toVector();

    double[] launchDirField = new double[3];
    double heading = robotPose.getRotation().getZ();
    rotateAboutZ(directionVectorRobotFrame, heading, launchDirField);

    double shootLen = SHOOTER_LENGTH.in(Meters);
    return new double[] {
      baseTranslation.get(X) + launchDirField[X] * shootLen,
      baseTranslation.get(Y) + launchDirField[Y] * shootLen,
      baseTranslation.get(Z) + launchDirField[Z] * shootLen
    };
  }

  protected static double[] launchVelocity(
      double wheelVelocity,
      double pitch,
      double yaw,
      Pose3d robotPose,
      ChassisSpeeds robotVelocity) {

    // Convert wheel velocity to shot speed and direction in field frame (avoid Rotation3d
    // allocations)
    double shotSpeed = launchSpeed(wheelVelocity);

    // Build unit direction in robot frame then rotate into field frame
    double[] launchDirRobot = new double[3];
    sphericalUnitVectorRobot(pitch, yaw, launchDirRobot);

    double heading = robotPose.getRotation().getZ();
    double[] launchDirField = new double[3];
    rotateAboutZ(launchDirRobot, heading, launchDirField);

    // shot velocity in field frame
    double[] shotVelocity = new double[3];
    shotVelocity[X] = launchDirField[X] * shotSpeed;
    shotVelocity[Y] = launchDirField[Y] * shotSpeed;
    shotVelocity[Z] = launchDirField[Z] * shotSpeed;

    return launchVelocity(shotVelocity, robotPose, robotVelocity);
  }

  protected static double[] launchVelocity(
      double[] shotVelocityField, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    // Convert shot direction (field frame) into robot-frame launch direction (for tip radius)
    double heading = robotPose.getRotation().getZ();

    // Direction in robot frame = rotate by -heading around Z
    double c = Math.cos(heading);
    double s = Math.sin(heading);
    double dirRx = shotVelocityField[X] * c + shotVelocityField[Y] * s;
    double dirRy = -shotVelocityField[X] * s + shotVelocityField[Y] * c;
    double dirRz = shotVelocityField[Z]; // Z unaffected by yaw rotation

    double[] dirRobot = new double[] {dirRx, dirRy, dirRz};
    double[] dirRobotUnit = new double[3];
    double n =
        Math.sqrt(
            dirRobot[X] * dirRobot[X] + dirRobot[Y] * dirRobot[Y] + dirRobot[Z] * dirRobot[Z]);
    if (n == 0.0) {
      dirRobotUnit[X] = dirRobotUnit[Y] = dirRobotUnit[Z] = 0.0;
      return shotVelocityField; // No direction, so no shooter velocity contribution.
    }
    dirRobotUnit[X] = dirRobot[X] / n;
    dirRobotUnit[Y] = dirRobot[Y] / n;
    dirRobotUnit[Z] = dirRobot[Z] / n;

    // Field-frame velocity of shooter due to robot motion (rotation + translation)
    double[] shooterVel = shooterVelocity(dirRobotUnit, robotPose, robotVelocity);

    // Combine shot muzzle velocity (in field) with shooter motion; Z is taken from
    // shotVelocityField
    return new double[] {
      shotVelocityField[X] + shooterVel[X],
      shotVelocityField[Y] + shooterVel[Y],
      shotVelocityField[Z] + shooterVel[Z]
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
