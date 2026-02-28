package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public class MovingShooting {
  public static double[] calculateLaunchParametersWhileMoving(
      double distance, Pose3d robotPose, ChassisSpeeds velocity) {
    double[] hub = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);
    double[] shooterPose = FuelVisualizer.robotToFuel(hub, robotPose);

    double yawStationary = Math.atan2(hub[X] - shooterPose[X], hub[Y] - shooterPose[Y]);

    double[] launchParameters = ShotOptimizer.distanceSpeedAndPitch(distance);
    double speed = launchParameters[ShotOptimizer.SPEED];

    double[] planarShotVelocityVector = {
      speed * Math.cos(yawStationary), speed * Math.sin(yawStationary)
    };
    double[] movingShotVelocity = {
      planarShotVelocityVector[X] - velocity.vxMetersPerSecond,
      planarShotVelocityVector[Y] - velocity.vyMetersPerSecond
    };

    return new double[] {
      distance,
      launchParameters[ShotOptimizer.SPEED],
      launchParameters[ShotOptimizer.PITCH],
      Math.atan2(movingShotVelocity[Y], movingShotVelocity[X])
    };
  }

  public static double[] launchVelocityWhileMoving(Pose3d robotPose, ChassisSpeeds velocity) {
    double distance = FuelVisualizer.distanceToHub(robotPose);
    double[] launchParameters =  calculateLaunchParametersWhileMoving(distance, robotPose, velocity);
    
    return FuelVisualizer.shotVelocity(launchParameters[ShotOptimizer.SPEED], launchParameters[ShotOptimizer.PITCH], launchParameters[ShotOptimizer.YAW], robotPose);
  }
}
