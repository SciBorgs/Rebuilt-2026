package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.*;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel.*;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.add3;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.norm3;
import static org.sciborgs1155.robot.commands.shooting.ShotOptimizer.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotGenerator {
  private static final double VELOCITY_DEADBAND = 0.1;

  private ShotGenerator() {}

  private static double[] stationaryLaunchParameters(double[] shooterPose, double heading) {
    double yDisplacement = GOAL[Y] - shooterPose[Y];
    double xDisplacement = GOAL[X] - shooterPose[X];

    double distance = Math.hypot(yDisplacement, xDisplacement);
    double yaw = Math.atan2(yDisplacement, xDisplacement) - heading;

    double[] launchParameters = TableGenerator.directLaunchParameters(distance);

    return new double[] {distance, launchParameters[SPEED], launchParameters[PITCH], yaw};
  }

  private static double[] movingLaunchParameters(
      double[] stationaryShotVelocity, double[] shooterVelocity, double heading) {
    double[] movingShotVelocity = sub3(stationaryShotVelocity, shooterVelocity);
    return launchParameters(movingShotVelocity, heading);
  }

  /**
   * Calculates the launch parameters to score FUEL in the HUB.
   *
   * @param robotPose the pose of the robot
   * @param robotVelocity the velocity of the robot
   * @return the launch parameters [DISTANCE, SPEED, PITCH, YAW]
   */
  public static double[] calculateLaunchParameters(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double heading = robotPose.getRotation().getZ();

    double[] shooterPose =
        add3(shooterPose(robotPose), fromTranslation(robotPose.getTranslation()));
    double[] stationaryLaunchParameters = stationaryLaunchParameters(shooterPose, heading);
    double[] stationaryShotVelocity = shotVelocity(stationaryLaunchParameters, heading);
    double[] shooterVelocity = shooterVelocity(stationaryShotVelocity, robotPose, robotVelocity);

    if (norm3(shooterVelocity) < VELOCITY_DEADBAND) return stationaryLaunchParameters;
    else return movingLaunchParameters(stationaryShotVelocity, shooterVelocity, heading);
  }
}
