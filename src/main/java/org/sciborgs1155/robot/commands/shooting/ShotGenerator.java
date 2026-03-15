package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.*;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;

public final class ShotGenerator {
  static final boolean DRAG_ENABLED = true;
  static final boolean LIFT_ENABLED = false;

  static final double CLEARANCE = 0.13;
  static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

  static final double SCORE_DEPTH = 0;
  static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

  static final double PITCH_PRECISION = Math.PI / 256;

  static final double MAX_SPEED = 20;
  static final double MIN_PITCH = Math.PI / 2 - MAX_ANGLE.in(Radians);
  static final double MAX_PITCH = Math.PI / 2 - MIN_ANGLE.in(Radians);

  static final double[] GOAL = fromTranslation(Hub.TOP_CENTER_POINT);

  private static double speedCache;

  private ShotGenerator() {}

  static double[] optimizedLaunchParameters(double distance) {
    double startingPitch = MIN_PITCH;

    for (double testPitch = startingPitch; testPitch < MAX_PITCH; testPitch += PITCH_PRECISION)
      if (ShotOptimizer.reaches(distance, new double[] {MAX_SPEED, testPitch, 0})) {
        double startingSpeed = speedCache == 0 ? MAX_SPEED : speedCache;
        double testSpeed = ShotOptimizer.optimize(distance, startingSpeed, testPitch);
        double[] launchParameters = new double[] {testSpeed, testPitch, 0};
        if (ShotOptimizer.clears(distance, launchParameters)) {
          speedCache = testSpeed;

          return new double[] {
            testSpeed, testPitch, 0, ShotOptimizer.error(distance, launchParameters)
          };
        }
      }

    return new double[] {0, 0, 0, 0};
  }

  /** Calculates the launch parameters required to shoot on the move (speed, pitch, yaw). */
  public static double[] movingLaunchParameters(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double heading = robotPose.getRotation().getZ();
    double[] robotToShooter = robotToShooter(heading);

    double x = GOAL[X] - robotToShooter[X] - robotPose.getX();
    double y = GOAL[Y] - robotToShooter[Y] - robotPose.getY();

    double distance = Math.sqrt(x * x + y * y);
    double yaw = Math.atan2(y, x) - heading;

    double[] directLaunchParameters = TableGenerator.directLaunchParameters(distance);
    double[] stationaryLaunchParameters = {
      directLaunchParameters[SPEED], directLaunchParameters[PITCH], yaw
    };

    double[] robotRelativeShotVelocity = robotRelativeShotVelocity(stationaryLaunchParameters);
    double[] stationaryShotVelocity = fieldRelative(robotRelativeShotVelocity, heading);
    double[] shooterVelocity =
        shooterVelocity(
            robotVelocity.vxMetersPerSecond,
            robotVelocity.vyMetersPerSecond,
            robotVelocity.omegaRadiansPerSecond,
            heading);

    return launchParameters(
        robotRelative(
            new double[] {
              stationaryShotVelocity[X] - shooterVelocity[X],
              stationaryShotVelocity[Y] - shooterVelocity[Y],
              stationaryShotVelocity[Z] - shooterVelocity[Z]
            },
            heading));
  }

  /**
   * Creates a FuelVisualizer with the settings used to generate shots for the shooting algorithm.
   */
  public static ProjectileVisualizer createVisualizer(Drive drive) {
    return fromLaunchParameters(
            () -> movingLaunchParameters(drive.pose3d(), drive.fieldRelativeChassisSpeeds()), drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(
            0.05,
            ShotOptimizer.MAX_AIR_TIME,
            ShotOptimizer.TRAJECTORY_RESOLUTION,
            ShotOptimizer.TRAJECTORY_RESOLUTION)
        .config(true, true);
  }
}
