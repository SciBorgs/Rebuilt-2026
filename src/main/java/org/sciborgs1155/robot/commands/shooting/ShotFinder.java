package org.sciborgs1155.robot.commands.shooting;

import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotFinder {

    private static final double[] HUB_POSE = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

    static public double findYawStationary(Pose3d pose) {

        double[] turretPose = FuelVisualizer.robotToFuel(HUB_POSE, pose);
        double xDiff = HUB_POSE[0] - turretPose[0];
        double yDiff = HUB_POSE[1] - turretPose[1];

        double stationaryYaw = Math.atan2(yDiff, xDiff);
        
        return stationaryYaw;

    }

    static public double findYawMoving(double yaw, ChassisSpeeds velocity, double distance, Pose3d pose) {

        double[] toHubVector = {ShotOptimizer.calculateLaunchParameters(distance)[1] * Math.cos(findYawStationary(pose)), 
                                ShotOptimizer.calculateLaunchParameters(distance)[1] * Math.sin(findYawStationary(pose))} ;
        double[] robotVelocityVector = {velocity.vxMetersPerSecond, velocity.vyMetersPerSecond};
        double[] netVector = {toHubVector[0] - robotVelocityVector[0], toHubVector[1] - robotVelocityVector[1]};

        double MovingYaw = Math.atan2(netVector[1], netVector[0]);

        return MovingYaw;

    }

}
