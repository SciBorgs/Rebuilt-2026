package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialRotationalVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterToInitial;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.MAX_AIR_TIME;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.MAX_OPTIMIZER_ITERATIONS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.MAX_TOF_ANALYSIS_ITERATIONS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.OPTIMIZATION_THRESHOLD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.PITCH_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.SPEED_KD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.SPEED_KP;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.TOF_ANALYSIS_THRESHOLD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.TOF_KD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.TOF_KP;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.DRAG_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.FUEL_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.LIFT_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CLEARANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CLEARANCE_CHECK;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.GOAL;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SCORE_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SCORE_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.TOF_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.TOF_RADIUS;

import edu.wpi.first.math.MathUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameters;

/**
 * A utility class that uses simulated trajectories and closed-loop iteration to estimate launch
 * parameters from shot data.
 */
public final class ShotOptimizer {
  /** Buffer used to store simulated trajectory. */
  private static double[][] trajectoryBuffer = new double[0][];

  /** Cached launch parameters used in trajectory simulation. */
  private static LaunchParameters launchParameterCache = new LaunchParameters(0, 0, 0);

  /** The FUEL object used to simulate trajectory. */
  private static Fuel fuel = new Fuel();

  // PREVENTS INSTANTIATION
  private ShotOptimizer() {}

  /**
   * Optimizes launch speed using a PD loop for maximum accuracy (intensive).
   *
   * @param launchParameters the launch parameters to optimize
   * @param scoreRadius the maximum planar distance from the HUB origin to count as 'scored'
   * @param scoreDepth the depth above/below the HUB rim to check for scoring
   */
  public static void optimizeForAccuracy(
      LaunchParameters launchParameters, double scoreRadius, double scoreDepth) {
    fuel.withScoringParameters(GOAL, scoreRadius, scoreDepth);
    fuel.config(RESOLUTION, true, DRAG_ENABLED, true, LIFT_ENABLED);

    double optimalSpeed =
        runSimplePDLoop(
            launchParameters.speed(),
            MAX_OPTIMIZER_ITERATIONS,
            OPTIMIZATION_THRESHOLD,
            speed -> {
              launchParameters.setSpeed(speed);
              return planarErrorFromHub(simulateTrajectory(launchParameters));
            },
            SPEED_KP,
            SPEED_KD,
            MIN_SPEED,
            MAX_SPEED);

    launchParameters.setSpeed(optimalSpeed);
  }

  /**
   * Optimizes launch speed using a PD loop to attain the specified time-of-flight (intensive).
   *
   * @param launchParameters the launch parameters to optimize
   * @param timeOfFlight the target time-of-flight in seconds
   */
  public static void optimizeForTimeOfFlight(
      LaunchParameters launchParameters, double timeOfFlight) {
    optimizeForAccuracy(launchParameters, TOF_RADIUS, TOF_DEPTH);

    double optimalSpeed =
        runSimplePDLoop(
            launchParameters.speed(),
            MAX_TOF_ANALYSIS_ITERATIONS,
            TOF_ANALYSIS_THRESHOLD,
            speed -> {
              launchParameters.setSpeed(speed);
              return timeOfFlight(simulateTrajectory(launchParameters)) - timeOfFlight;
            },
            TOF_KP,
            TOF_KD,
            MIN_SPEED,
            MAX_SPEED);

    launchParameters.setSpeed(optimalSpeed);
  }

  /**
   * Returns the optimal launch parameters for the given distance (minimal airtime). Clear cache
   * when generating a new table or doing a single optimization.
   *
   * @param launchParameters the planar distance of the shooter from the HUB in meters
   */
  public static void optimizeForAirTime(LaunchParameters launchParameters) {
    double increment = Math.PI * 2 / PITCH_RESOLUTION;
    LaunchParameters maxLaunchParameters =
        new LaunchParameters(launchParameters.distance(), MAX_SPEED, MIN_PITCH);

    for (double testPitch = MIN_PITCH; testPitch <= MAX_PITCH; testPitch += increment) {
      maxLaunchParameters.setPitch(testPitch);
      if (!reachesHub(simulateTrajectory(maxLaunchParameters))) continue;

      launchParameters.setPitch(testPitch);
      optimizeForAccuracy(launchParameters, SCORE_RADIUS, SCORE_DEPTH);
      if (clearsRimHeight(simulateTrajectory(launchParameters))) return;
    }

    throw new UnsupportedOperationException("Impossible shot!");
  }

  /**
   * Whether or not the projectile is able to reach the planar target origin when launched with the
   * given parameters.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static boolean reachesHub(double[][] trajectory) {
    double[] finalTranslation = finalTranslation(trajectory);
    return !(finalTranslation[X] - GOAL[X] < 0 || finalTranslation[Z] < FUEL_RADIUS);
  }

  /**
   * The final planar distance from the target origin when launched with the given parameters.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static double planarErrorFromHub(double[][] trajectory) {
    return finalTranslation(trajectory)[X] - GOAL[X];
  }

  /**
   * The time-of-flight of the FUEL when launched with the given parameters.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static double timeOfFlight(double[][] trajectory) {
    return trajectory.length * 1.0 / RESOLUTION;
  }

  /**
   * Whether or not the projectile clears the specified height over the rim of the target when
   * launched with the given parameters.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static boolean clearsRimHeight(double[][] trajectory) {
    for (int index = trajectory.length - 1; index >= 0; index--) {
      double[] translation = trajectory[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) return false;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  /**
   * The final translation of the FUEL launched with the given parameters.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static double[] finalTranslation(double[][] trajectory) {
    if (trajectory.length > 0) return trajectory[trajectory.length - 1];
    else return new double[] {0, 0, 0};
  }

  /**
   * Generates a trajectory with the given parameters and stores it in the buffer.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   * @param launchParameters the launch parameters for the shot
   */
  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  public static double[][] simulateTrajectory(LaunchParameters launchParameters) {
    if (launchParameters.equals(launchParameterCache)) return trajectoryBuffer;
    if (launchParameters.isOutOfBounds())
      throw new UnsupportedOperationException("Cannot simulate out-of-bounds trajectory!");

    double distance = launchParameters.distance();
    double speed = launchParameters.speed();
    double pitch = launchParameters.pitch();

    launchParameterCache = new LaunchParameters(distance, speed, pitch);

    fuel.reset();
    final List<double[]> poseList = new ArrayList<>(512);

    double[] shotVelocity = robotRelativeShotVelocity(speed, pitch, 0);
    double[] shooterToInitial = shooterToInitial(pitch, 0, 0);
    double[] shooterTranslation = {GOAL[X] - distance, GOAL[Y], ROBOT_TO_SHOOTER[Z]};
    double initialRotationalVelocity = initialRotationalVelocity();

    fuel.initialize(
        new double[] {
          shooterTranslation[X] + shooterToInitial[X],
          shooterTranslation[Y] + shooterToInitial[Y],
          shooterTranslation[Z] + shooterToInitial[Z]
        },
        shotVelocity,
        new double[3],
        initialRotationalVelocity);

    double maxFrames = RESOLUTION * MAX_AIR_TIME;
    for (int frames = 0; frames <= maxFrames; frames++) {
      poseList.add(new double[] {fuel.x, fuel.y, fuel.z});
      fuel.step();
      if (fuel.willScore() || fuel.willMiss()) break;
    }

    trajectoryBuffer = poseList.toArray(new double[0][]);
    return trajectoryBuffer;
  }

  /**
   * Runs a simple PD loop on the variable.
   *
   * @param initialValue the initial value of the variable
   * @param maxIterations the maximum number of loop iterations to run
   * @param threshold the error threshold that once reached, will end the loop
   * @param errorFunction a function of the variable that returns the error
   * @param proportionalConstant the proportional constant in the PD controller
   * @param derivativeConstant the derivative constant in the PD controller
   * @param minimum the minimum value of the variable
   * @param maximum the maximum value of the variable
   */
  private static double runSimplePDLoop(
      double initialValue,
      int maxIterations,
      double threshold,
      DoubleFunction<Double> errorFunction,
      double proportionalConstant,
      double derivativeConstant,
      double minimum,
      double maximum) {
    double value = initialValue;
    double previousError = errorFunction.apply(initialValue);

    for (int iteration = 0; iteration <= maxIterations; iteration++) {
      double error = errorFunction.apply(value);
      if (Math.abs(error) <= threshold) return value;

      double proportional = proportionalConstant * -error;
      double derivative = (iteration == 0) ? 0 : derivativeConstant * (previousError - error);

      value = MathUtil.clamp(value + proportional + derivative, minimum, maximum);
      previousError = error;
    }

    return value;
  }
}
