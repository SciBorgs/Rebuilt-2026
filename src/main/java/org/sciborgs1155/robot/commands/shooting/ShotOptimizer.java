package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.FUEL_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialRotationalVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterToInitial;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.diff;
import static org.sciborgs1155.robot.commands.shooting.ShotGenerator.*;

import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public final class ShotOptimizer {
  static final double SPEED_KP = 0.5;
  static final double SPEED_KD = 0.05;

  static final int OPTIMIZATION_RESOLUTION = 300;
  static final double OPTIMIZATION_THRESHOLD = 0.01;

  static final double MAX_AIR_TIME = 10;
  static final int TRAJECTORY_RESOLUTION = 200;
  static final double MAX_FRAMES = TRAJECTORY_RESOLUTION * MAX_AIR_TIME;

  private static double[] launchParameterCache = new double[3];
  private static double[][] trajectoryBuffer = new double[0][];
  private static Projectile projectile =
      new Fuel()
          .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
          .config(TRAJECTORY_RESOLUTION, true, DRAG_ENABLED, false, LIFT_ENABLED);

  private ShotOptimizer() {}

  /** Calculates the launch speed given a launch pitch and a planar distance from the target. */
  public static double optimize(double distance, double startingSpeed, double pitch) {
    double lastDisplacement = distance;
    double optimalSpeed = startingSpeed;

    for (int iterations = 0; iterations < OPTIMIZATION_RESOLUTION; iterations++) {
      generateDirectTrajectory(distance, new double[] {optimalSpeed, pitch, 0});
      double finalDisplacement = trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];
      if (Math.abs(finalDisplacement) < OPTIMIZATION_THRESHOLD) return optimalSpeed;

      double proportional = SPEED_KP * -finalDisplacement;
      double derivative = SPEED_KD * (lastDisplacement - finalDisplacement);
      if (lastDisplacement == distance) derivative = 0;

      optimalSpeed = optimalSpeed + proportional + derivative;
      lastDisplacement = finalDisplacement;
    }

    return optimalSpeed;
  }

  static void generateDirectTrajectory(double distance, double[] launchParameters) {
    if (!diff(launchParameters, launchParameterCache)) return;
    launchParameterCache = launchParameters.clone();

    projectile.reset();
    final List<double[]> poseList = new ArrayList<>();

    double[] shotVelocity = robotRelativeShotVelocity(launchParameters);
    double[] shooterToInitial = shooterToInitial(shotVelocity, 0);
    double[] shooterTranslation = {GOAL[X] - distance, GOAL[Y], ROBOT_TO_SHOOTER.getZ()};

    double initialRotationalVelocity = initialRotationalVelocity();

    projectile.launch(
        new double[] {
          shooterTranslation[X] + shooterToInitial[X],
          shooterTranslation[Y] + shooterToInitial[Y],
          shooterTranslation[Z] + shooterToInitial[Z]
        },
        shotVelocity,
        new double[3],
        initialRotationalVelocity);

    for (int frames = 0; frames < MAX_FRAMES; frames++) {
      poseList.add(new double[] {projectile.x, projectile.y, projectile.z});
      projectile.periodic();

      if (projectile.willScore()) break;
      if (projectile.willMiss()) break;
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

  static double error(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);
    return trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];
  }
}
