package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.norm3;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.sub3;
import static org.sciborgs1155.robot.commands.shooting.ShotOptimizer.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MovingShooting {
  public static double[] calculateLaunchParameters(Pose3d robotPose, ChassisSpeeds velocity) {
    double heading = robotPose.getRotation().getZ();
    double[] shooterPose = FuelVisualizer.robotToShooter(robotPose);
    double yaw = Math.atan2(GOAL[Y] - shooterPose[Y], GOAL[X] - shooterPose[X]);

    double distance = norm3(sub3(GOAL, shooterPose));

    double speed = ShotTable.speed(distance);
    double pitch = ShotTable.angle(distance);

    return new double[] {distance, speed, pitch, yaw - heading};
  }
}
