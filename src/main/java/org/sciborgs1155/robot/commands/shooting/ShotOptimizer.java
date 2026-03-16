package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialRotationalVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterToInitial;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.diff;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.*;

import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public final class ShotOptimizer {
  private static double speedCache;
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

  public static double[] optimizedLaunchParameters(double distance) {
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

  private static void generateDirectTrajectory(double distance, double[] launchParameters) {
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

  public static boolean clears(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);

    for (int index = trajectoryBuffer.length - 1; index >= 0; index--) {
      double[] translation = trajectoryBuffer[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) return false;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  public static boolean reaches(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);

    double[] maxDistance = trajectoryBuffer[trajectoryBuffer.length - 1];
    return !(maxDistance[X] - GOAL[X] < 0 || maxDistance[Z] < FUEL_RADIUS);
  }

  public static double error(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);
    return trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];
  }
}
