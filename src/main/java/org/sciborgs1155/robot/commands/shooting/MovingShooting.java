package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ShotOptimizer.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MovingShooting {
  public static double[] calculateLaunchParameters(Pose3d robotPose, ChassisSpeeds velocity) {
    double heading = robotPose.getRotation().getZ();
    double[] robotToShooter = FuelVisualizer.robotToShooter(robotPose);
    double[] shooterPose = {
      robotToShooter[X] + robotPose.getX(), robotToShooter[Y] + robotPose.getY()
    };

    double yDisplacement = GOAL[Y] - shooterPose[Y];
    double xDisplacement = GOAL[X] - shooterPose[X];

    double yaw = Math.atan2(yDisplacement, xDisplacement);
    double distance = Math.hypot(yDisplacement, xDisplacement);

    double speed = ShotTable.speed(distance);
    double pitch = ShotTable.angle(distance);

    return new double[] {distance, speed, pitch, yaw - heading};
  }
}
