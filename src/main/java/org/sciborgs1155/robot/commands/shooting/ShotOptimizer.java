package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.*;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel.*;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.SHOOTING_ANGLE_OFFSET;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public class ShotOptimizer {
  private static final boolean DRAG_ENABLED = true;
  private static final boolean TORQUE_ENABLED = false;
  private static final boolean LIFT_ENABLED = false;

  private static final int TRAJECTORY_RESOLUTION = 100;
  private static final int OPTIMIZATION_RESOLUTION = 500;
  private static final int TRAJECTORY_SIZE_LIMIT = 200;

  private static final double CLEARANCE = 0.13;
  private static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

  private static final double SCORE_TOLERANCE = 0;
  private static final double SCORE_DEPTH = 0;

  protected static final double[] GOAL = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

  private static final double SPEED_CONSTANT = 0.01;
  private static final double ANGLE_INCREMENT = Math.PI / 12;

  private static final double MAX_SPEED = 20;
  protected static final double MAXIMUM_ANGLE =
      SHOOTING_ANGLE_OFFSET.in(Radians) - MIN_ANGLE.in(Radians);
  protected static final double MINIMUM_ANGLE =
      SHOOTING_ANGLE_OFFSET.in(Radians) - MAX_ANGLE.in(Radians);

  private static final Projectile projectile =
      new Fuel()
          .withScoringParameters(SCORE_TOLERANCE, SCORE_DEPTH)
          .config(TRAJECTORY_RESOLUTION, true, DRAG_ENABLED, TORQUE_ENABLED, LIFT_ENABLED);

  protected static double[] directLaunchParameters(double distance) {
    double speed = 0;
    double angle = 0;

    for (double testAngle = MINIMUM_ANGLE;
        testAngle < MAXIMUM_ANGLE;
        testAngle += ANGLE_INCREMENT) {

      // TEST IF SHOT IS POSSIBLE
      if (!checkAngle(distance, testAngle)) continue;

      double optimalSpeed = optimizeSpeed(distance, MAX_SPEED, testAngle);
      double optimalAngle = testAngle;

      // TEST IF FUEL HAS ENOUGH CLEARANCE OVER THE HUB EDGE
      if (!checkClearance(distance, optimalSpeed, optimalAngle)) continue;

      speed = optimalSpeed;
      angle = optimalAngle;
      break;
    }

    return new double[] {distance, speed, angle, 0};
  }

  private static double optimizeSpeed(double distance, double startingSpeed, double angle) {
    int iterations = 0;
    double speed = startingSpeed;

    while (iterations < OPTIMIZATION_RESOLUTION) {
      double[][] trajectory = directTrajectory(new double[] {distance, speed, angle, 0});

      double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDisplacement > 0) speed -= SPEED_CONSTANT * finalDistance;
      if (finalDisplacement < 0) speed += SPEED_CONSTANT * finalDistance;
      iterations++;
    }

    return speed;
  }

  private static boolean checkAngle(double distance, double angle) {
    double[][] trajectory = directTrajectory(new double[] {distance, MAX_SPEED, angle, 0});
    double[] finalTranslation = trajectory[trajectory.length - 1];

    return !(finalTranslation[X] - GOAL[X] < 0 || finalTranslation[Z] < GOAL[Z]);
  }

  private static boolean checkClearance(double distance, double speed, double angle) {
    double[][] trajectory = directTrajectory(new double[] {distance, speed, angle, 0});
    for (int index = trajectory.length - 1; index >= 0; index--) {
      double[] translation = trajectory[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) break;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  private static double[][] directTrajectory(double[] launchParameters) {
    projectile.reset();
    List<double[]> poseList = new ArrayList<>();

    Pose3d robotPose =
        new Pose3d(
            GOAL[X] - launchParameters[DISTANCE] - ROBOT_TO_SHOOTER.getX(),
            GOAL[Y] - ROBOT_TO_SHOOTER.getY(),
            0,
            new Rotation3d());

    double[] shotVelocity = shotVelocity(launchParameters, robotPose.getRotation().getZ());
    double[] launchVelocity = launchVelocity(shotVelocity, robotPose, new ChassisSpeeds());
    double[] launchTranslation = launchTranslation(shotVelocity, robotPose);

    projectile.launch(launchTranslation, launchVelocity, new double[4], 0);

    int frames = 0;
    while (!projectile.willMiss() && !projectile.willScore()) {
      frames++;
      if (frames >= TRAJECTORY_SIZE_LIMIT) break;

      poseList.add(projectile.translation.clone());
      projectile.periodic();
    }

    return poseList.toArray(new double[0][]);
  }

  public static Command displayOptimizedShot(double distance) {
    return Commands.runOnce(
        () -> {
          double[] launchParameters = directLaunchParameters(distance);
          double[][] trajectory = directTrajectory(launchParameters);
          Pose3d[] poses = new Pose3d[trajectory.length];

          for (int index = 0; index < poses.length; index++)
            poses[index] = toPose(trajectory[index], 0);

          LoggingUtils.log("Shooting/ShotOptimizerDisplay", poses, Pose3d.struct);
        });
  }
}
