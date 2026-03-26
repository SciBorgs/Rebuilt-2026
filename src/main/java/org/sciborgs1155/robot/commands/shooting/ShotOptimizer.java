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
import org.sciborgs1155.robot.shooter.ShooterConstants;

/**
 * A utility class used to generate accurate launch parameters for launches from a given distance.
 */
public final class ShotOptimizer {
  /** Cached speed value used in 'optimizedLaunchParameters' method. */
  private static double speedCache;

  /** Cached launch parameters used in 'generateDirectTrajectory' method. */
  private static double[] launchParameterCache = new double[3];

  private static double[][] trajectoryBuffer = new double[0][];
  private static Projectile projectile =
      new Fuel()
          .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
          .config(OPTIMIZER_RESOLUTION, true, DRAG_ENABLED, false, LIFT_ENABLED);

  private ShotOptimizer() {}

  /** Calculates the launch speed given a launch pitch and a planar distance from the target. */
  public static double optimizedSpeed(double distance, double startingSpeed, double pitch) {
    double previousError = distance;
    double optimalSpeed = startingSpeed;

    for (int iterations = 0; iterations <= MAX_OPTIMIZER_ITERATIONS; iterations++) {
      generateDirectTrajectory(distance, new double[] {optimalSpeed, pitch, 0});
      double error = trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];
      if (Math.abs(error) <= OPTIMIZATION_THRESHOLD) return optimalSpeed;

      double proportional = SPEED_KP * -error;
      double derivative = SPEED_KD * (previousError - error);
      if (previousError == distance) derivative = 0;

      optimalSpeed += (proportional + derivative);
      previousError = error;
    }

    return optimalSpeed;
  }

  /** Calculates the launch speed given the pitch and time-of-flight of a successful shot. */
  public static double estimateSpeed(double distance, double pitch, double timeOfFlight) {
    double previousError = timeOfFlight;
    double speed = optimizedSpeed(distance, MAX_SPEED, pitch);

    for (int iterations = 0; iterations <= MAX_TOF_ANALYSIS_ITERATIONS; iterations++) {
      generateDirectTrajectory(distance, new double[] {speed, pitch, 0});
      double error = trajectoryBuffer.length * 1.0 / OPTIMIZER_RESOLUTION - timeOfFlight;
      if (Math.abs(error) < TOF_ANALYSIS_THRESHOLD) return speed;

      double proportional = TOF_KP * -error;
      double derivative = TOF_KD * (previousError - error);
      if (previousError == timeOfFlight) derivative = 0;

      speed += (proportional + derivative);
      previousError = error;
    }

    return speed;
  }

  /**
   * Returns the optimal launch parameters for the given distance (minimal airtime). Clear cache
   * when generating a new table or doing a single optimization.
   */
  public static double[] optimizedLaunchParameters(double distance) {
    double increment = Math.PI * 2 / PITCH_RESOLUTION;
    double startingPitch = MIN_PITCH;

    for (double testPitch = startingPitch; testPitch <= MAX_PITCH; testPitch += increment)
      if (reaches(distance, new double[] {MAX_SPEED, testPitch, 0})) {
        double startingSpeed = speedCache == 0 ? MAX_SPEED : speedCache;
        double testSpeed = optimizedSpeed(distance, startingSpeed, testPitch);
        double[] launchParameters = {testSpeed, testPitch, 0};

        if (clears(distance, launchParameters)) {
          speedCache = testSpeed;

          return new double[] {testSpeed, testPitch, error(distance, launchParameters)};
        }
      }

    return new double[] {0, 0, 0, 0};
  }

  /**
   * Whether or not the projectile is able to reach the planar target origin when launched with the
   * given parameters.
   */
  public static boolean reaches(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);

    double[] maxDistance = trajectoryBuffer[trajectoryBuffer.length - 1];
    return !(maxDistance[X] - GOAL[X] < 0 || maxDistance[Z] < FUEL_RADIUS);
  }

  /** The final planar distance from the target origin when launched with the given parameters. */
  public static double error(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);
    return trajectoryBuffer[trajectoryBuffer.length - 1][X] - GOAL[X];
  }

  /**
   * Whether or not the projectile clears the specified height over the rim of the target when
   * launched with the given parameters.
   */
  public static boolean clears(double distance, double[] launchParameters) {
    generateDirectTrajectory(distance, launchParameters);

    for (int index = trajectoryBuffer.length - 1; index >= 0; index--) {
      double[] translation = trajectoryBuffer[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) return false;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  /** Clears cached speed and trajectory. */
  public static void clearCache() {
    speedCache = 0;
    launchParameterCache = new double[3];
  }

  static double[][] buffer() {
    return trajectoryBuffer.clone();
  }

  static void generateDirectTrajectory(double distance, double[] launchParameters) {
    if (!diff(launchParameters[X], launchParameterCache[X])
        && !diff(launchParameters[Y], launchParameterCache[Y])
        && !diff(launchParameters[Z], launchParameterCache[Z])) return;

    System.arraycopy(launchParameters, 0, launchParameterCache, 0, 3);

    projectile.reset();
    final List<double[]> poseList = new ArrayList<>();

    double[] shotVelocity = robotRelativeShotVelocity(launchParameters);
    double[] shooterToInitial = shooterToInitial(launchParameters[PITCH], launchParameters[YAW], 0);
    double[] shooterTranslation = {
      GOAL[X] - distance, GOAL[Y], ShooterConstants.CENTER_TO_SHOOTER.getZ()
    };

    double initialRotationalVelocity = initialRotationalVelocity();

    projectile.initialize(
        new double[] {
          shooterTranslation[X] + shooterToInitial[X],
          shooterTranslation[Y] + shooterToInitial[Y],
          shooterTranslation[Z] + shooterToInitial[Z]
        },
        shotVelocity,
        new double[3],
        initialRotationalVelocity);

    double maxFrames = OPTIMIZER_RESOLUTION * MAX_AIR_TIME;
    for (int frames = 0; frames <= maxFrames; frames++) {
      poseList.add(new double[] {projectile.x, projectile.y, projectile.z});
      projectile.step();

      if (projectile.willScore()) break;
      if (projectile.willMiss()) break;
    }

    trajectoryBuffer = poseList.toArray(new double[0][]);
  }
}
