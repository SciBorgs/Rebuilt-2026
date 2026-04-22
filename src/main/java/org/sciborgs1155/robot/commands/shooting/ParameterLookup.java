package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.DEGREE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.PARAMETER_INCREMENT;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.parameterLookupSelector;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.PolynomialRegression;
import org.sciborgs1155.lib.PolynomialRegression.ModelSelector;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.LookupID;
import org.sciborgs1155.robot.commands.shooting.ShotOptimizer.DirectLaunchParameters;

/**
 * A class used to analyze and compile shot data into a polynomial regression model that can be used
 * to determine the direct launch parameters for a given distance.
 */
public final class ParameterLookup {
  private static LookupID lookupID = LookupID.MAXIMUM_SPEED;
  private static final ExecutorService executor =
      Executors.newSingleThreadExecutor(runnable -> new Thread(runnable, "Parameter Lookup"));

  /** The progress of the current generation, in entries. */
  private static int progress;

  /** Whether the current generation has finished or not. */
  private static boolean done;

  // PREVENTS INSTANTIATION
  private ParameterLookup() {}

  /**
   * Creates a command to generate the lookup model.
   *
   * @param lookupID the identifier for the lookup model
   * @param function takes in a mutable launch parameter object and modifies it to store the launch
   *     parameters to be used by the lookup.
   */
  public static Command startGeneration(
      LookupID lookupID, Consumer<DirectLaunchParameters> function) {
    return Commands.runOnce(
            () -> executor.submit(() -> addLookup(lookupID, createLookup(function))))
        .andThen(Commands.idle().until(() -> done));
  }

  /** Logs data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/Parameter Lookup/INDEX", lookupID);
    LoggingUtils.log(
        "Shooting/Parameter Lookup/LOADED", parameterLookupSelector.containsKey(lookupID));
    LoggingUtils.log("Shooting/Parameter Lookup/Generation/PROGRESS", progress);
    LoggingUtils.log("Shooting/Parameter Lookup/Generation/COMPLETE", done);
  }

  /**
   * The launch speed of the FUEL according to the lookup (meters per second).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double speed(double distance) {
    return parameterLookupSelector.containsKey(lookupID)
        ? parameterLookupSelector.get(lookupID).speed(distance)
        : 0;
  }

  /**
   * The launch pitch of the FUEL according to the lookup (radians).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double pitch(double distance) {
    return parameterLookupSelector.containsKey(lookupID)
        ? parameterLookupSelector.get(lookupID).pitch(distance)
        : 0;
  }

  /**
   * Switches the lookup model used by the speed and pitch methods.
   *
   * @param lookupID the identifier for the lookup model being loaded
   */
  public static void useLookup(LookupID lookupID) {
    ParameterLookup.lookupID = lookupID;
  }

  /**
   * Adds a launch parameter lookup model to the selector.
   *
   * @param lookupID the identifier for the lookup model being added
   * @param lookup the lookup model to add to the selector
   */
  public static void addLookup(LookupID lookupID, LaunchParameterLookup lookup) {
    parameterLookupSelector.put(lookupID, lookup);
  }

  /**
   * Creates a launch parameter lookup from a distance function.
   *
   * @param function takes in a mutable launch parameter object and modifies it to store the launch
   *     parameters to be used by the lookup.
   */
  public static LaunchParameterLookup createLookup(Consumer<DirectLaunchParameters> function) {
    DirectLaunchParameters launchParameters =
        new DirectLaunchParameters(MIN_DISTANCE, MAX_SPEED, Math.PI / 4);

    done = false;
    progress = 0;
    double[][] dataTable =
        ModelSelector.dataTable(
            distance -> {
              progress++;
              launchParameters.setDistance(distance);
              function.accept(launchParameters);
              return new double[] {
                launchParameters.distance(), launchParameters.speed(), launchParameters.pitch()
              };
            },
            MIN_DISTANCE,
            MAX_DISTANCE,
            PARAMETER_INCREMENT);

    PolynomialRegression speed = ModelSelector.regression(dataTable, DISTANCE, SPEED, DEGREE);
    PolynomialRegression pitch = ModelSelector.regression(dataTable, DISTANCE, PITCH, DEGREE);

    double speedR2 = ModelSelector.rSquared(speed, dataTable, DISTANCE, SPEED);
    double pitchR2 = ModelSelector.rSquared(pitch, dataTable, DISTANCE, PITCH);

    done = true;

    LaunchParameterLookup lookup =
        new LaunchParameterLookup(
            speed.getCoefficients(), pitch.getCoefficients(), MIN_DISTANCE, MAX_DISTANCE);

    LoggingUtils.log(
        "Shooting/Parameter Lookup/Generation/Speed/Coefficients", speed.getCoefficients());
    LoggingUtils.log(
        "Shooting/Parameter Lookup/Generation/Pitch/Coefficients", pitch.getCoefficients());
    LoggingUtils.log("Shooting/Parameter Lookup/Generation/Speed/R^2", speedR2);
    LoggingUtils.log("Shooting/Parameter Lookup/Generation/Pitch/R^2", pitchR2);

    return lookup;
  }

  /** Combines polynomial regression models into a singular parameter lookup. */
  public static class LaunchParameterLookup {
    private final double minDistance, maxDistance;
    private final double[] speedCoefficients;
    private final double[] pitchCoefficients;

    /**
     * Combines polynomial regression models into a singular parameter lookup.
     *
     * @param speedRegression the polynomial coefficients to model launch speed as a function of
     *     distance
     * @param pitchRegression the polynomial coefficients to model launch pitch as a function of
     *     distance
     * @param minDistance the minimum distance from the HUB, in meters
     * @param maxDistance the maximum distance from the HUB, in meters
     */
    public LaunchParameterLookup(
        double[] speedCoefficients,
        double[] pitchCoefficients,
        double minDistance,
        double maxDistance) {
      this.speedCoefficients = speedCoefficients.clone();
      this.pitchCoefficients = pitchCoefficients.clone();

      this.minDistance = minDistance;
      this.maxDistance = maxDistance;
    }

    /**
     * The launch speed of the FUEL (meters per second).
     *
     * @param distance the planar distance from the HUB to the shooter's origin, in meters
     */
    public double speed(double distance) {
      if (tooFar(distance)) return evaluate(MAX_DISTANCE, speedCoefficients);
      if (tooClose(distance)) return evaluate(MIN_DISTANCE, speedCoefficients);
      return evaluate(distance, speedCoefficients);
    }

    /** The coefficients for the speed regression model. */
    public double[] speedCoefficients() {
      return speedCoefficients.clone();
    }

    /**
     * The launch pitch of the FUEL (radians).
     *
     * @param distance the planar distance from the HUB to the shooter's origin, in meters
     */
    public double pitch(double distance) {
      if (tooFar(distance)) return evaluate(MAX_DISTANCE, pitchCoefficients);
      if (tooClose(distance)) return evaluate(MIN_DISTANCE, pitchCoefficients);
      return evaluate(distance, pitchCoefficients);
    }

    /** The coefficients for the pitch regression model. */
    public double[] pitchCoefficients() {
      return pitchCoefficients.clone();
    }

    /**
     * Whether or not the specified distance is above the lookup range.
     *
     * @param distance the distance from the HUB, in meters
     */
    public boolean tooFar(double distance) {
      return distance > maxDistance;
    }

    /**
     * Whether or not the specified distance is below the lookup range.
     *
     * @param distance the distance from the HUB, in meters
     */
    public boolean tooClose(double distance) {
      return distance < minDistance;
    }

    /**
     * Evaluates the value of a polynomial at a specified x value.
     *
     * @param x the x value to input into the polynomial
     * @param coefficients an array of coefficients describing the polynomial
     */
    public static double evaluate(double x, double[] coefficients) {
      double result = 0;
      for (int degree = 0; degree < coefficients.length; degree++)
        result += coefficients[degree] * Math.pow(x, degree);
      return result;
    }
  }
}
