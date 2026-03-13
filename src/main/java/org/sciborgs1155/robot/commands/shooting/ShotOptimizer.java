package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.FUEL_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialRotation;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialRotationalVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ShotGenerator.*;

import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public final class ShotOptimizer {
  static final double SPEED_KP = 0.005;
  static final double SPEED_KD = 0.05;

  private static double[][] trajectoryBuffer = new double[0][];
  private static Projectile projectile =
      new Fuel()
          .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
          .config(TRAJECTORY_RESOLUTION, true, DRAG_ENABLED, false, LIFT_ENABLED);

  private ShotOptimizer() {}

  /** Calculates the launch speed given a launch pitch and a planar distance from the target. */
  public static double optimize(double distance, double pitch) {
    double lastDisplacement = distance;
    double optimalSpeed = MAX_SPEED;

    for (int iterations = 0; iterations < OPTIMIZATION_RESOLUTION; iterations++) {
      generateDirectTrajectory(distance, new double[] {optimalSpeed, pitch, 0});
      double finalDisplacement = trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];

      double proportional = SPEED_KP * -finalDisplacement;
      double derivative = SPEED_KD * (lastDisplacement - finalDisplacement);
      if (lastDisplacement == distance) derivative = 0;

      optimalSpeed = optimalSpeed + proportional + derivative;
      lastDisplacement = finalDisplacement;
    }

    return optimalSpeed;
  }

  static void generateDirectTrajectory(double distance, double[] launchParameters) {
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

  static boolean clears(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);

    for (int index = trajectoryBuffer.length - 1; index >= 0; index--) {
      double[] translation = trajectoryBuffer[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) return false;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  static boolean reaches(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);

    double[] maxDistance = trajectoryBuffer[trajectoryBuffer.length - 1];
    return !(maxDistance[X] - GOAL[X] < 0 || maxDistance[Z] < FUEL_RADIUS);
  }

  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  static double[][] trajectory() {
    return trajectoryBuffer;
  }
}
