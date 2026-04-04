package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.AIR_TIME_MODEL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameterRegressionModel;

/**
 * A class used to analyze and compile shot data into a polynomial regression model that can be used
 * to determine the direct launch parameters for a given distance.
 */
public final class ParameterLookup {
  private static int modelID = AIR_TIME_MODEL;
  private static final HashMap<Integer, LaunchParameterRegressionModel> modelSelector = new HashMap<>();
  private static final ExecutorService executor =
      Executors.newSingleThreadExecutor(runnable -> new Thread(runnable, "Parameter Lookup"));

  // PREVENTS INSTANTIATION
  private ParameterLookup() {}

  public static Command generateLookup() {
    return Commands.runOnce(() -> executor.execute(ParameterLookup::airTimeLookup));
  }

  private static void airTimeLookup() {
    modelSelector.put(AIR_TIME_MODEL, LaunchParameterRegressionModel.createLookup(
      (distance, launchParameters) -> {
        launchParameters.setDistance(distance);
        ShotOptimizer.optimizeForAirTime(launchParameters);
        return new double[]{launchParameters.distance(), launchParameters.speed(), launchParameters.pitch(), 0};
      }));
  }
  
  /** Logs data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/Model/INDEX", modelID);
    LoggingUtils.log("Shooting/Model/LOADED", modelSelector.containsKey(modelID));
  }

  /**
   * The launch speed of the FUEL from the given distance estimated using the model (meters per
   * second).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double speed(double distance) {
    return modelSelector.containsKey(modelID) ? modelSelector.get(modelID).speed(distance) : 0;
  }

  /**
   * The launch pitch of the FUEL from the given distance estimated using the model (radians).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double pitch(double distance) {
    return modelSelector.containsKey(modelID) ? modelSelector.get(modelID).pitch(distance) : 0;
  }

  /**
   * The horizontal error of a launch from the given distance using the model (meters). Simulation
   * ends when the FUEL either hits the ground or hits the horizontal plane formed by the rim of the
   * hub while going downwards.
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double error(double distance) {
    return modelSelector.containsKey(modelID) ? modelSelector.get(modelID).error(distance) : 0;
  }
}
