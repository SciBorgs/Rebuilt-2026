package org.sciborgs1155.lib.shooting;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  double calculate(Translation3d pose, ChassisSpeeds velocity);

  double translationX = 5;
  double translationY = 20;
  double hubX = 4.0345;
  double hubY = 3.057;
  double hubZ = 1.827276;
  double shooterZ = 0.254;

  public static double calculateVelocity(Drive drive) {
    InterpolatingDoubleTreeMap velocityLookup = new InterpolatingDoubleTreeMap();

    // have magnus force + drag incorporated
    velocityLookup.put(2, 781.2063637 - drive.robotRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(2.5, 651.4441804 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(3, 628.0489819 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(3.5, 630.7480891 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(4, 643.4548917 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(4.5, 660.6486686 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(5, 680.1591875 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(5.5, 700.8373228 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(6, 722.0939247 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(
        6.121768808, 727.3166827 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(7.2898, 777.6028308 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(7.5, 786.617125 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(8, 807.9529887 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(8.5, 829.1007001 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(9, 850.0372997 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(9.5, 870.7462475 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(10, 891.2216084 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(10.5, 911.4618775 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(11, 931.4696372 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(11.5, 951.2425991 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(12, 970.7926058 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
    velocityLookup.put(
        12.5831802, 991.3452656 - drive.fieldRelativeChassisSpeeds().vxMetersPerSecond);
  }

  public static double calculatePosition(Pose2d position) {

    Translation3d pose =
        new Translation3d(position.getX() + translationX, position.getX() + translationY, shooterZ);
    Translation3d hub = new Translation3d(hubX, hubY, hubZ);

    double hDistance = Math.sqrt(Math.pow(hubX - pose.getX(), 2) + Math.pow(hubY - pose.getY(), 2));
    double vDistance = hubZ - shooterZ;

    LinearVelocity fuelVelocity = MetersPerSecond.of(5);

    Angle currentXangle =
        Radians.of(90 / (Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2))));
    Angle neededXAngle =
        Radians.of(90 / (Math.sqrt(Math.pow(hub.getX(), 2) + Math.pow(hub.getY(), 2))));
    return neededXAngle.minus(currentXangle).in(Radians);
  }
}
