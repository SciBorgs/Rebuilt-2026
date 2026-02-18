package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;

import org.sciborgs1155.lib.projectiles.TrajectoryVisualizer;
import org.sciborgs1155.robot.FieldConstants.Hub;

public class BasedShootingAlgorithm {
  TrajectoryVisualizer e;
  public double[] fieldRelativeShotVelocityVector(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double rotationalSpeed = robotVelocity.omegaRadiansPerSecond * ROBOT_TO_SHOOTER.toTranslation2d().getNorm();
    double tangentialDirection = robotPose.getRotation().getZ() + Math.PI / 2.0;

    double xShooterVelocity = robotVelocity.vxMetersPerSecond + rotationalSpeed * Math.cos(tangentialDirection);
    double yShooterVelocity = robotVelocity.vyMetersPerSecond + rotationalSpeed * Math.sin(tangentialDirection);

    double xDistance = Hub.TOP_CENTER_POINT.getX() - robotPose.getX();
    double yDistance = Hub.TOP_CENTER_POINT.getY() - robotPose.getY();
    double planarDistance = Math.hypot(xDistance, yDistance);

    double xRelativeVelocity = xDistance / planarDistance;
    double yRelativeVelocity = yDistance / planarDistance;
    double planarVelocity = 1;

    return new double[]{xRelativeVelocity * planarVelocity - xShooterVelocity, yRelativeVelocity * planarVelocity - yShooterVelocity, 5};
  }
}
