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
import java.util.function.DoubleFunction;

import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

/**
 * A utility class used to generate accurate launch parameters for launches from a given distance.
 */
public final class ShotOptimizer {
  /** Cached speed value used for optimization. */
  private static double speedCache;

  /** Cached launch parameters used in simulating trajectory. */
  private static double[] launchParameterCache = new double[3];

  /** Buffer used to store simulated trajectory. */
  private static double[][] trajectoryBuffer = new double[0][];
  private static Projectile projectile =
      new Fuel()
          .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
          .config(OPTIMIZER_RESOLUTION, true, DRAG_ENABLED, false, LIFT_ENABLED);

  private ShotOptimizer() {}

  /**
   * Calculates the launch speed given a launch pitch and a planar distance from the target.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param startingSpeed the speed to start the optimization process from (closer values yield
   *     faster results)
   * @param pitch the launch pitch of the FUEL in radians
   */
  public static double optimizeForAccuracy(double distance, double startingSpeed, double pitch) {
    return runPID(
      startingSpeed,
      finalTranslation(distance, new double[]{startingSpeed, pitch, 0})[X] - GOAL[X],
      MAX_OPTIMIZER_ITERATIONS,
      OPTIMIZATION_THRESHOLD,
      speed -> finalTranslation(distance, new double[]{speed, pitch, 0})[X] - GOAL[X],
      SPEED_KP,
      SPEED_KD);
  }

  /**
   * Estimates the launch speed given the pitch and time-of-flight of a successful shot.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param pitch the launch pitch of the FUEL in radians
   * @param timeOfFlight the time from the FUEL losing contact with the robot to the time it cross
   *     the plane formed by the rim of the HUB
   */
  public static double estimateSpeed(double distance, double pitch, double timeOfFlight) {
    return runPID(
      optimizeForAccuracy(distance, MAX_SPEED, pitch),
      timeOfFlight(distance, new double[]{MAX_SPEED, pitch, 0}),
      MAX_TOF_ANALYSIS_ITERATIONS,
      TOF_ANALYSIS_THRESHOLD,
      speed -> timeOfFlight(distance, new double[]{speed, pitch, 0}),
      TOF_KP,
      TOF_KD);
  }

  /**
   * Returns the optimal launch parameters for the given distance (minimal airtime). Clear cache
   * when generating a new table or doing a single optimization.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   */
  public static double[] optimizeForAirTime(double distance) {
    double increment = Math.PI * 2 / PITCH_RESOLUTION;
    double startingPitch = MIN_PITCH;

    for (double testPitch = startingPitch; testPitch <= MAX_PITCH; testPitch += increment)
      if (canReachHub(distance, new double[] {MAX_SPEED, testPitch, 0})) {
        double startingSpeed = speedCache == 0 ? MAX_SPEED : speedCache;
        double testSpeed = optimizeForAccuracy(distance, startingSpeed, testPitch);
        double[] launchParameters = {testSpeed, testPitch, 0};

        if (clearsRimHeight(distance, launchParameters)) {
          speedCache = testSpeed;

          return new double[] {testSpeed, testPitch, planarErrorFromHub(distance, launchParameters)};
        }
      }

    return new double[] {0, 0, 0, 0};
  }

  /**
   * Whether or not the projectile is able to reach the planar target origin when launched with the
   * given parameters.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters in the format implied by 'ShootingConstants'
   */
  public static boolean canReachHub(double distance, double[] launchParameters) {
    double[] finalTranslation = finalTranslation(distance, launchParameters);
    return !(finalTranslation[X] - GOAL[X] < 0 || finalTranslation[Z] < FUEL_RADIUS);
  }

  /**
   * The final planar distance from the target origin when launched with the given parameters.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters in the format implied by 'ShootingConstants'
   */
  public static double planarErrorFromHub(double distance, double[] launchParameters) {
    return finalTranslation(distance, launchParameters)[X] - GOAL[X];
  }

  /**
   * The time-of-flight of the FUEL when launched with the given parameters.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters in the format implied by 'ShootingConstants'
   */
  public static double timeOfFlight(double distance, double[] launchParameters) {
    return simulateTrajectory(distance, launchParameters).length * 1.0 / OPTIMIZER_RESOLUTION;
  }

  /**
   * Whether or not the projectile clears the specified height over the rim of the target when
   * launched with the given parameters.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters in the format implied by 'ShootingConstants'
   */
  public static boolean clearsRimHeight(double distance, double[] launchParameters) {
    simulateTrajectory(distance, launchParameters);

    for (int index = trajectoryBuffer.length - 1; index >= 0; index--) {
      double[] translation = trajectoryBuffer[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) return false;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  /** The current trajectory stored in the buffer. */
  public static double[][] buffer() {
    return trajectoryBuffer.clone();
  }

  /** Clears cached speed and trajectory. */
  public static void clearCache() {
    speedCache = 0;
  }

  /**
   * The final translation of the FUEL launched with the given parameters.
   * 
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters in the format implied by 'ShootingConstants'
   */
  public static double[] finalTranslation(double distance, double[] launchParameters) {
    return simulateTrajectory(distance, launchParameters)[trajectoryBuffer.length - 1];
  }

  /**
   * Generates a trajectory with the given parameters.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters in the format implied by 'ShootingConstants'
   */
  public static double[][] simulateTrajectory(double distance, double[] launchParameters) {
    if (!diff(launchParameters[X], launchParameterCache[X])
        && !diff(launchParameters[Y], launchParameterCache[Y])
        && !diff(launchParameters[Z], launchParameterCache[Z])) return trajectoryBuffer;

    System.arraycopy(launchParameters, 0, launchParameterCache, 0, 3);

    projectile.reset();
    final List<double[]> poseList = new ArrayList<>();

    double[] shotVelocity = robotRelativeShotVelocity(launchParameters);
    double[] shooterToInitial = shooterToInitial(launchParameters[PITCH], launchParameters[YAW], 0);
    double[] shooterTranslation = {GOAL[X] - distance, GOAL[Y], ROBOT_TO_SHOOTER[Z]};
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
    return trajectoryBuffer;
  }

  private static double runPID(
      double initialValue,
      double initialError,
      int maxIterations,
      double threshold,
      DoubleFunction<Double> errorFunction,
      double kp,
      double kd) {
    double value = initialValue;
    double previousError = initialError;

    for (int iteration = 0; iteration <= maxIterations; iteration++) {
      double error = errorFunction.apply(value);
      if (Math.abs(error) <= threshold) return value;

      double proportional = kp * -error;
      double derivative = (previousError == initialError) ? 0 : kd * (previousError - error);

      value += proportional + derivative;
      previousError = error;
    }

    return value;
  }
}
