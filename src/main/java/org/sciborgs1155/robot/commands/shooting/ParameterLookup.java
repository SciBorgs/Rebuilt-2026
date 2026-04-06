package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.INCREMENT;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.lookupSelector;
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
import org.sciborgs1155.lib.PolynomialRegression.ModelSelector.RegressionModel;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.DirectLaunchParameters;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameterLookup;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.LookupID;

/**
 * A class used to analyze and compile shot data into a polynomial regression model that can be used
 * to determine the direct launch parameters for a given distance.
 */
public final class ParameterLookup {
  private static LookupID lookupID = LookupID.MINIMAL_AIR_TIME;
  private static final ExecutorService executor =
      Executors.newSingleThreadExecutor(runnable -> new Thread(runnable, "Parameter Lookup"));
  private static int progress;
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
  public static Command startGeneration(LookupID lookupID, Consumer<DirectLaunchParameters> function) {
    return Commands.runOnce(() -> executor.submit(() -> addLookup(lookupID, createLookup(function)))).andThen(Commands.idle().until(() -> done));
  }

  /** Logs data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/Model/INDEX", lookupID);
    LoggingUtils.log("Shooting/Model/LOADED", lookupSelector.containsKey(lookupID));
    LoggingUtils.log("Shooting/Model/PROGRESS", progress);

    if (!lookupSelector.containsKey(lookupID)) return;
    LoggingUtils.log("Shooting/Model/SPEED", lookupSelector.get(lookupID).speedRegression());
    LoggingUtils.log("Shooting/Model/PITCH", lookupSelector.get(lookupID).pitchRegression());
  }

  /**
   * The launch speed of the FUEL from the given distance estimated using the model (meters per
   * second).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double speed(double distance) {
    return lookupSelector.containsKey(lookupID) ? lookupSelector.get(lookupID).speed(distance) : 0;
  }

  /**
   * The launch pitch of the FUEL from the given distance estimated using the model (radians).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double pitch(double distance) {
    return lookupSelector.containsKey(lookupID) ? lookupSelector.get(lookupID).pitch(distance) : 0;
  }

  /**
   * Adds a launch parameter lookup to the selector.
   *
   * @param lookupID the identifier for the lookup model being added
   * @param lookup the lookup model to add to the selector
   */
  public static void addLookup(LookupID lookupID, LaunchParameterLookup lookup) {
    lookupSelector.put(lookupID, lookup);
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
            INCREMENT);

    PolynomialRegression speed = ModelSelector.regression(dataTable, DISTANCE, SPEED, 5);
    PolynomialRegression pitch = ModelSelector.regression(dataTable, DISTANCE, PITCH, 5);

    done = true;
    return new LaunchParameterLookup(
        new RegressionModel(speed.getDegree(), speed.getCoefficients()),
        new RegressionModel(pitch.getDegree(), pitch.getCoefficients()));
  }
}
