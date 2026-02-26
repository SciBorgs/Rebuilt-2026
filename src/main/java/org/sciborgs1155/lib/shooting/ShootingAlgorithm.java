package org.sciborgs1155.lib.shooting;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import org.sciborgs1155.robot.drive.*;

@FunctionalInterface
public interface ShootingAlgorithm {

  /**
   * Calculates the direction and speed to run the shooter at to shoot accurately towards the goal.
   * This should take into account both the position of the shooter and the movement of the shooter.
   *
   * @param pose The current field-relative position of the shooter. This is a Translation3d because
   *     the shooter may be offset from the center of the robot.
   * @param velocity The current translational velocity of the shooter.
   * @return The direction and speed to run the shooter to shoot accurately towards the goal.
   */
  double translationX = 5;

  double translationY = 20;
  double hubX = 4.0345;
  double hubY = 3.057;
  double hubZ = 1.827276;
  double shooterZ = 0.254;

  double vDistance = hubZ - shooterZ;

  public static void calculateVelocity(Drive drive) {
    InterpolatingDoubleTreeMap velocityLookup = new InterpolatingDoubleTreeMap();
    // have magnus force + drag incorporated, in rpm

    velocityLookup.put(1.0, 412.8068811 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(1.5, 445.8391575 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(2.0, 485.7590902 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(2.5, 534.4106828 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(3.0, 586.7989146 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(3.5, 646.4667232 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(4.0, 708.7900973 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(4.5, 774.6490016 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(5.0, 844.2642488 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(5.5, 913.1253041 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(6.0, 995.2547422 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(
        6.121768808, 1012.107345 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(7.2898, 1192.260842 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(7.5, 1220.460545 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(8.0, 1309.570282 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(8.5, 1393.760008 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(9.0, 1476.205005 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(9.5, 1563.032657 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(10.0, 1657.262536 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(10.5, 1748.499412 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(11.0, 1846.872435 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(11.5, 1936.610605 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(12.0, 2040.275821 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(
        12.5831802, 2146.433823 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
  }

  public static void calculateHoodAngle() {
    InterpolatingDoubleTreeMap angleLookup = new InterpolatingDoubleTreeMap();

    angleLookup.put(1.0, 1.234);
    angleLookup.put(1.5, 1.085);
    angleLookup.put(2.0, 0.965);
    angleLookup.put(2.5, 0.86);
    angleLookup.put(3.0, 0.775);
    angleLookup.put(3.5, 0.7);
    angleLookup.put(4.0, 0.638);
    angleLookup.put(4.5, 0.585);
    angleLookup.put(5.0, 0.539);
    angleLookup.put(5.5, 0.501);
    angleLookup.put(6.0, 0.463);
    angleLookup.put(6.121768808, 0.456);
    angleLookup.put(7.2898, 0.394);
    angleLookup.put(7.5, 0.386);
    angleLookup.put(8.0, 0.363);
    angleLookup.put(8.5, 0.344);
    angleLookup.put(9.0, 0.3275);
    angleLookup.put(9.5, 0.312);
    angleLookup.put(10.0, 0.297);
    angleLookup.put(10.5, 0.284);
    angleLookup.put(11.0, 0.2714);
    angleLookup.put(11.5, 0.261);
    angleLookup.put(12.0, 0.2501);
    angleLookup.put(12.5831802, 0.24);
  }

  public static double turretAngle(Pose2d position) {
    Translation3d pose =
        new Translation3d(position.getX() + translationX, position.getX() + translationY, shooterZ);
    Translation3d hub = new Translation3d(hubX, hubY, hubZ);

    double hDistance = Math.sqrt(Math.pow(hubX - pose.getX(), 2) + Math.pow(hubY - pose.getY(), 2));

    LinearVelocity fuelVelocity = MetersPerSecond.of(5);

    Angle currentXangle =
        Radians.of(90 / (Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2))));
    Angle neededXAngle =
        Radians.of(90 / (Math.sqrt(Math.pow(hub.getX(), 2) + Math.pow(hub.getY(), 2))));

    return neededXAngle.minus(currentXangle).in(Radians);
  }

  double turretAngle = turretAngle(odometry);
  double hoodAngle = angleLookup.interpolate(hDistance);
  double velocity = velocityLookup.interpolate(hDistance);

  ShootingAlgorithm calculate(double turretAngle, double hoodAngle, double velocity);
}
