package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.lib.ProjectileVisualizer.X;
import static org.sciborgs1155.lib.ProjectileVisualizer.Y;
import static org.sciborgs1155.lib.ProjectileVisualizer.Z;
import static org.sciborgs1155.robot.Constants.EPS;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.initialRotationalVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterToInitial;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.MAX_AIR_TIME;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.MAX_OPTIMIZER_ITERATIONS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.MAX_TOF_ANALYSIS_ITERATIONS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.OPTIMIZATION_THRESHOLD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.PITCH_KD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.PITCH_KP;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.PITCH_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.SPEED_KD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.SPEED_KP;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OptimizerConstants.SPEED_RESOLUTION;
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
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.DirectLaunchParameters;

/**
 * A utility class that uses simulated trajectories and closed-loop iteration to estimate launch
 * parameters from shot data.
 */
public final class ShotOptimizer {
  /** Buffer used to store simulated trajectory. */
  private static double[][] trajectoryBuffer = new double[0][];

  /** Cached launch parameters used in trajectory simulation. */
  private static DirectLaunchParameters launchParameterCache = new DirectLaunchParameters(0, 0, 0);

  /** The FUEL object used to simulate trajectory. */
  private static Fuel fuel =
      (Fuel) new Fuel().config(RESOLUTION, true, DRAG_ENABLED, true, LIFT_ENABLED);

  // PREVENTS INSTANTIATION
  private ShotOptimizer() {}

  /**
   * Optimizes launch speed using a PD loop for maximum accuracy (intensive).
   *
   * @param launchParameters the launch parameters to optimize
   * @param scoreRadius the maximum planar distance from the HUB origin to count as 'scored'
   * @param scoreDepth the depth above/below the HUB rim to check for scoring
   */
  public static void optimizeSpeedForAccuracy(
      DirectLaunchParameters launchParameters, double scoreRadius, double scoreDepth) {
    fuel.withScoringParameters(GOAL, scoreRadius, scoreDepth);

    runSimplePDLoop(
        launchParameters.speed(),
        MAX_OPTIMIZER_ITERATIONS,
        OPTIMIZATION_THRESHOLD,
        speed -> {
          launchParameters.setSpeed(speed);
          return hubError(simulateTrajectory(launchParameters));
        },
        SPEED_KP,
        SPEED_KD,
        MIN_SPEED,
        MAX_SPEED);
  }

  /**
   * Optimizes launch pitch using a PD loop for maximum accuracy (intensive).
   *
   * @param launchParameters the launch parameters to optimize
   * @param scoreRadius the maximum planar distance from the HUB origin to count as 'scored'
   * @param scoreDepth the depth above/below the HUB rim to check for scoring
   */
  public static void optimizePitchForAccuracy(
      DirectLaunchParameters launchParameters, double scoreRadius, double scoreDepth) {
    fuel.withScoringParameters(GOAL, scoreRadius, scoreDepth);

    runSimplePDLoop(
        launchParameters.pitch(),
        MAX_OPTIMIZER_ITERATIONS,
        OPTIMIZATION_THRESHOLD,
        pitch -> {
          launchParameters.setPitch(pitch);
          return -hubError(simulateTrajectory(launchParameters));
        },
        PITCH_KP,
        PITCH_KD,
        MIN_PITCH,
        MAX_PITCH);
  }

  /**
   * Optimizes launch speed using a PD loop to attain the specified time-of-flight (intensive).
   *
   * @param launchParameters the launch parameters to optimize
   * @param timeOfFlight the target time-of-flight in seconds
   */
  public static void optimizeSpeedForTimeOfFlight(
      DirectLaunchParameters launchParameters, double timeOfFlight) {
    optimizeSpeedForAccuracy(launchParameters, TOF_RADIUS, TOF_DEPTH);

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
  }

  /**
   * Modifies the given launch parameters in-place to minimize air time (lowest pitch).
   *
   * @param launchParameters the launch parameters to optimize
   */
  public static void optimizeForAirTime(DirectLaunchParameters launchParameters) {
    double increment = Math.PI * 2 / PITCH_RESOLUTION;
    
    double linePitch = Math.atan((GOAL[Z] - ROBOT_TO_SHOOTER[Z]) / launchParameters.distance());
    double startingPitch = Math.max(linePitch, MIN_PITCH);

    for (double testPitch = startingPitch; testPitch <= MAX_PITCH; testPitch += increment) {
      launchParameters.setPitch(testPitch);
      optimizeSpeedForAccuracy(launchParameters, SCORE_RADIUS, SCORE_DEPTH);

      double[][] trajectory = simulateTrajectory(launchParameters);
      if (Math.abs(hubError(trajectory)) > OPTIMIZATION_THRESHOLD) continue;
      if (clearsRimHeight(trajectory)) return;
    }

    throw new UnsupportedOperationException("Impossible shot!");
  }

  /**
   * Modifies the given launch parameters in-place to maximize accuracy (highest pitch).
   * 
   * @param launchParameters the launch parameters to optimize
   */
  public static void optimizeForAccuracy(DirectLaunchParameters launchParameters) {
    double increment = (MAX_PITCH - MIN_PITCH) / PITCH_RESOLUTION;

    for (double testPitch = MAX_PITCH; testPitch >= MIN_PITCH; testPitch -= increment) {
      launchParameters.setPitch(testPitch);
      optimizeSpeedForAccuracy(launchParameters, SCORE_RADIUS, SCORE_DEPTH);

      double[][] trajectory = simulateTrajectory(launchParameters);
      if (Math.abs(hubError(trajectory)) > OPTIMIZATION_THRESHOLD) continue;
      if (clearsRimHeight(trajectory)) return;
    }

    throw new UnsupportedOperationException("Impossible shot!");
  }

  /**
   * Modifies the given launch parameters in-place to maximize throughput (highest speed).
   *
   * @param launchParameters the launch parameters to optimize
   */
  public static void optimizeForSpeed(DirectLaunchParameters launchParameters) {
    double increment = (MAX_SPEED - MIN_SPEED) / SPEED_RESOLUTION;

    for (double testSpeed = MAX_SPEED; testSpeed >= MIN_SPEED; testSpeed -= increment) {
      launchParameters.setSpeed(testSpeed);
      optimizePitchForAccuracy(launchParameters, SCORE_RADIUS, SCORE_DEPTH);

      double[][] trajectory = simulateTrajectory(launchParameters);
      if (Math.abs(hubError(trajectory)) > OPTIMIZATION_THRESHOLD) continue;
      if (clearsRimHeight(trajectory)) return;
    }

    throw new UnsupportedOperationException("Impossible shot!");
  }


  /**
   * The time-of-flight of the FUEL when launched with the given parameters (seconds).
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static double timeOfFlight(double[][] trajectory) {
    return trajectory.length * 1.0 / RESOLUTION;
  }

  /**
   * The planar error from the HUB (meters). Positive value indicates overshoot.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static double hubError(double[][] trajectory) {
    return finalTranslation(trajectory)[X] - GOAL[X];
  }

  /**
   * Whether or not the projectile clears the specified height over the rim of the HUB.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static boolean clearsRimHeight(double[][] trajectory) {
    for (int index = trajectory.length - 1; index >= 0; index--) {
      double[] translation = trajectory[index];
      double distance = Math.abs(translation[X] - GOAL[X]);
      double verticalDistance = translation[Z] - GOAL[Z];

      if (distance >= CLEARANCE_CHECK) return verticalDistance > CLEARANCE;
    }

    return false;
  }

  /**
   * Whether or not the projectile fails to reach the HUB plane.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static boolean endsOnGround(double[][] trajectory) {
    return Math.abs(finalTranslation(trajectory)[Z] - FUEL_RADIUS) <= EPS;
  }

  /**
   * The final translation of the FUEL.
   *
   * @param trajectory the trajectory of the shot (array of [X, Y, Z] coordinates)
   */
  public static double[] finalTranslation(double[][] trajectory) {
    if (trajectory.length > 0) return trajectory[trajectory.length - 1];
    else return new double[] {0, 0, 0};
  }

  /**
   * Generates a trajectory with the given parameters and stores it in the buffer.
   *
   * @param launchParameters the launch parameters for the shot
   */
  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  public static double[][] simulateTrajectory(DirectLaunchParameters launchParameters) {
    if (launchParameters.equals(launchParameterCache)) return trajectoryBuffer;
    if (launchParameters.isOutOfBounds())
      throw new UnsupportedOperationException("Cannot simulate out-of-bounds trajectory!");

    double distance = launchParameters.distance();
    double speed = launchParameters.speed();
    double pitch = launchParameters.pitch();

    launchParameterCache = new DirectLaunchParameters(distance, speed, pitch);

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

    int maxFrames = (int) (RESOLUTION * MAX_AIR_TIME);
    for (int frames = 0; frames < maxFrames; frames++) {
      poseList.add(new double[] {fuel.x(), fuel.y(), fuel.z()});
      fuel.step();
      if (fuel.willScore() || fuel.willMiss()) break;
    }

    trajectoryBuffer = poseList.toArray(new double[0][]);
    return trajectoryBuffer;
  }

  /**
   * Generates and displays a trajectory with the given parameters.
   *
   * @param launchParameters the launch parameters for the shot
   */
  public static void displayTrajectory(DirectLaunchParameters launchParameters) {
    double distance = launchParameters.distance();
    double speed = launchParameters.speed();
    double pitch = launchParameters.pitch();

    fuel.reset();
    final List<Pose3d> poseList = new ArrayList<>(512);

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
    for (int frames = 0; frames < maxFrames; frames++) {
      poseList.add(fuel.pose());
      fuel.step();
      if (fuel.willScore() || fuel.willMiss()) break;
    }

    LoggingUtils.log(
        "Shooting/ShotOptimizerDisplay", poseList.toArray(new Pose3d[0]), Pose3d.struct);
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
    double previousError = 0;

    for (int iteration = 0; iteration < maxIterations; iteration++) {
      double error = errorFunction.apply(value);
      if (Math.abs(error) <= threshold) return value;

      double proportional = proportionalConstant * -error;
      double derivative = (iteration == 0) ? 0 : derivativeConstant * (previousError - error);

      value = MathUtil.clamp(value + proportional + derivative, minimum, maximum);
      
      if (error == previousError) return value;
      previousError = error;
    }

    errorFunction.apply(value);
    return value;
  }
}
