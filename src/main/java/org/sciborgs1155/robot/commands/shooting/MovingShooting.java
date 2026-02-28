package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.norm3;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.sub3;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public class MovingShooting {
  public static double[] calculateLaunchParameters(Pose3d robotPose, ChassisSpeeds velocity) {
    double[] hub = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);
    double[] shooterPose = FuelVisualizer.robotToFuel(new double[] {0, 0, 0}, robotPose);

    double yawStationary = Math.atan2(hub[Y] - shooterPose[Y], hub[X] - shooterPose[X]);

    double distance = norm3(sub3(hub, shooterPose));
    double[] launchParameters = ShotOptimizer.distanceSpeedAndPitch(distance);

    return new double[] {
      distance,
      launchParameters[ShotOptimizer.SPEED],
      launchParameters[ShotOptimizer.PITCH],
      yawStationary
    };
  }
}
