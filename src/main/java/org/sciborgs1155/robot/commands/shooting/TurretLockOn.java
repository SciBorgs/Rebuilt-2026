package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.atan2;
import static java.lang.Math.sin;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_ANGULAR_ACCEL;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_ACCELERATION;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Robot;    
import org.sciborgs1155.robot.FieldConstants.Hub;

public class TurretLockOn {

  private static double t = 10 * PERIOD.in(Seconds);

  /**
   * attempts to preemptively get the velocity of the robot through raw joystick inputs (same as Robot.java)
   * 
   * @param rawX
   * @param rawY
   * @param driver
   * @return
   */
  public static ChassisSpeeds joystickChassisSpeeds(
      InputStream rawX, InputStream rawY, CommandXboxController driver) {

    InputStream theta = InputStream.atan(rawX, rawY);

    InputStream x =
        Robot.r(rawX, rawY)
            .scale(theta.map(Math::cos))
            .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));
    InputStream y =
        Robot.r(rawX, rawY)
            .scale(theta.map(Math::sin))
            .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond));

    ChassisSpeeds robotChassisSpeeds =
        new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), Robot.omega(driver).getAsDouble());

    return robotChassisSpeeds;
  }

  /**
   * finds the projected pose2d of the robot based on its current pose, heading, and chassisSpeed
   * 
   * @param chassisSpeed 
   * @param robotPose
   * @param currentTheta
   * @return a Pose2d of the projected pose
   */
  public static Pose2d projectPose(
    ChassisSpeeds chassisSpeed, Pose2d robotPose, Rotation2d currentTheta) {
  double vX = chassisSpeed.vxMetersPerSecond;
  double vY = chassisSpeed.vyMetersPerSecond;
  double omega = chassisSpeed.omegaRadiansPerSecond;

  double projectedX = robotPose.getX() - ((vX * t) + ((1 / 2) * MAX_ACCELERATION * (t * t)));
  double projectedY = robotPose.getY() - ((vY * t) + ((1 / 2) * MAX_ACCELERATION * (t * t)));
  double projectedOmega =
      robotPose.getRotation().getRadians() + ((omega * t) + ((1 / 2) * MAX_ANGULAR_ACCEL.magnitude() * (t * t)));

  Pose2d projected = new Pose2d(projectedX, projectedY, new Rotation2d(projectedOmega));

  return projected;
}

  // given a chassisSpeed and robot pose (goal is just hub pos ig)
  // find the angular vel of turret to turn to position it must go to

  public static double FindTurretAngularVelocity(
      ChassisSpeeds chassisSpeed, Pose2d projectedPose, Rotation2d currentTheta) {

    double goalX = Hub.INNER_CENTER_POINT.getX();
    double goalY = Hub.INNER_CENTER_POINT.getY();

    double dX = goalX - projectPose(chassisSpeed, projectedPose, currentTheta).getX();
    double dY = goalY - projectPose(chassisSpeed, projectedPose, currentTheta).getY();

    double goalTheta = atan2(dY, dX);

    double dTheta = currentTheta.getRadians() - goalTheta;

    double angularVelocity = dTheta / t;

    return angularVelocity;
  }


}
