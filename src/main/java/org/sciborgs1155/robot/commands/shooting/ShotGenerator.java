package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
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
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;
import org.sciborgs1155.robot.drive.Drive;

public final class ShotGenerator {
  private static final boolean DRAG_ENABLED = true;
  private static final boolean LIFT_ENABLED = false;

  private static final double MAX_AIR_TIME = 10;
  private static final int TRAJECTORY_RESOLUTION = 100;
  private static final int OPTIMIZATION_RESOLUTION = 500;

  private static final double CLEARANCE = 0.13;
  private static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

  private static final double SCORE_DEPTH = 0;
  private static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

  private static final double SPEED_PRECISION = 0.005;
  private static final double PITCH_PRECISION = Math.PI / 128;

  private static double[][] trajectoryBuffer = new double[0][];

  static final double MAX_SPEED = 20;
  static final double MIN_PITCH = Math.PI / 2 - MAX_ANGLE.in(Radians);
  static final double MAX_PITCH = Math.PI / 2 - MIN_ANGLE.in(Radians);

  static final double[] GOAL = fromTranslation(Hub.TOP_CENTER_POINT);

  private static final Projectile projectile =
      new Fuel()
          .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
          .config(TRAJECTORY_RESOLUTION, true, DRAG_ENABLED, false, LIFT_ENABLED);

  private ShotGenerator() {}

  private static void generateDirectTrajectory(double distance, double[] launchParameters) {
    projectile.reset();
    final List<double[]> poseList = new ArrayList<>();

    double[] shotVelocity = robotRelativeShotVelocity(launchParameters);
    double[] initialVelocity = initialVelocity(shotVelocity, 0, 0, 0, 0);
    double[] initialTranslation = {GOAL[X] - distance, GOAL[Y], ROBOT_TO_SHOOTER.getZ()};
    double[] initialRotation = initialRotation(shotVelocity, 0);
    double initialRotationalVelocity = initialRotationalVelocity();

    projectile.launch(
        initialTranslation, initialVelocity, initialRotation, initialRotationalVelocity);

    int frames = 0;
    double maxFrames = TRAJECTORY_RESOLUTION * MAX_AIR_TIME;
    while (!projectile.willMiss() && !projectile.willScore() && frames < maxFrames) {
      poseList.add(new double[] {projectile.x, projectile.y, projectile.z});
      projectile.periodic();
      frames++;
    }

    trajectoryBuffer = poseList.toArray(new double[0][]);
  }

  private static double optimizeForSpeed(double distance, double speed, double angle) {
    for (int iterations = 0; iterations < OPTIMIZATION_RESOLUTION; iterations++) {
      generateDirectTrajectory(distance, new double[] {speed, angle, 0});
      double finalDisplacement = trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDisplacement > 0) speed -= SPEED_PRECISION * finalDistance;
      if (finalDisplacement < 0) speed += SPEED_PRECISION * finalDistance;
    }

    return speed;
  }

  protected static double[] optimizedLaunchParameters(double distance) {
    for (double testPitch = MIN_PITCH; testPitch < MAX_PITCH; testPitch += PITCH_PRECISION) {
      // CHECK IF SHOT IS POSSIBLE
      generateDirectTrajectory(distance, new double[] {MAX_SPEED, testPitch, 0});
      double[] maxDistance = trajectoryBuffer[trajectoryBuffer.length - 1];
      if (maxDistance[X] - GOAL[X] < 0 || maxDistance[Z] < FUEL_RADIUS) continue;

      // OPTIMIZE SHOT
      double speed = optimizeForSpeed(distance, MAX_SPEED, testPitch);

      // CHECK CLEARANCE
      generateDirectTrajectory(distance, new double[] {speed, testPitch, 0});

      boolean cleared = false;
      for (int index = trajectoryBuffer.length - 1; index >= 0; index--) {
        double[] translation = trajectoryBuffer[index];

        if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) break;
        if (translation[Z] > GOAL[Z] + CLEARANCE) cleared = true;
      }

      // RETURN FIRST ACCEPTABLE SHOT
      if (!cleared) continue;
      return new double[] {speed, testPitch, 0};
    }

    return new double[] {0, 0, 0};
  }

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

  public static ProjectileVisualizer createVisualizer(Drive drive) {
    return fromLaunchParameters(
            () -> movingLaunchParameters(drive.pose3d(), drive.fieldRelativeChassisSpeeds()), drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(0.05, MAX_AIR_TIME, TRAJECTORY_RESOLUTION * 30, TRAJECTORY_RESOLUTION)
        .config(true, true);
  }
}
