package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.DELIMITER;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.DISTANCE_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.ERROR;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.FORMAT;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.MAX_ERROR;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.MAX_LOOKUP_TABLE_SIZE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.PARAMETER_TABLE_PATH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_DIRECTORY;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Scanner;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameters;

/**
 * A utility class used to generate a distance/speed/pitch/error lookup table which can be
 * referenced when calculating launch parameters for shooting.
 */
public final class ParameterTable {
  private static InterpolatingDoubleTreeMap speedLookup = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap pitchLookup = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap errorLookup = new InterpolatingDoubleTreeMap();

  private static boolean status;
  private static boolean generationComplete;
  private static double averageError;

  private static int entriesGenerated;
  private static int entriesLoaded;

  private static ScheduledExecutorService executor =
      Executors.newScheduledThreadPool(
          1, runnable -> new Thread(runnable, "Parameter Lookup Table Generation"));

  private ParameterTable() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command generate() {
    return Commands.runOnce(
            () ->
                executor.submit(
                    () ->
                        generateTable(
                            PARAMETER_TABLE_PATH, MIN_DISTANCE, MAX_DISTANCE, DISTANCE_RESOLUTION)))
        .andThen(Commands.idle())
        .until(() ->  generationComplete);
  }

  private static void generateTable(String name, double min, double max, double resolution) {
    entriesGenerated = 0;
    double increment = 1 / resolution;

    ShotOptimizer.clearCache();
    Path path = Path.of(TABLE_DIRECTORY + "%s.ankit".formatted(name));

    generationComplete = false;
    try (BufferedWriter writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8)) {

      for (double distance = max;
          distance >= min && entriesGenerated <= MAX_LOOKUP_TABLE_SIZE;
          distance -= increment) {
        double[] entry = ShotOptimizer.optimizeForAirTime(distance);

        double speed = entry[LaunchParameters.SPEED];
        double pitch = entry[LaunchParameters.PITCH];
        double error = entry[LaunchParameters.ERROR];

        if (speed > MAX_SPEED
            || speed < MIN_SPEED
            || pitch < MIN_PITCH
            || pitch > MAX_PITCH
            || error > MAX_ERROR) continue;

        writer.write(FORMAT.formatted(distance, speed, pitch, error));
        writer.newLine();
        entriesGenerated++;
      }
    } catch (IOException exception) {
      exception.printStackTrace();
    }

    generationComplete = true;
  }

  /**
   * Loads the lookup table from the resources folder.
   *
   * @return a command to load the lookup table
   */
  public static Command load() {
    return Commands.runOnce(() -> loadTable(PARAMETER_TABLE_PATH));
  }

  private static void loadTable(String name) {
    speedLookup.clear();
    pitchLookup.clear();
    errorLookup.clear();

    double totalError = 0;
    Path path = Path.of(TABLE_DIRECTORY + "%s.ankit".formatted(name));

    try (Scanner scanner = new Scanner(path, StandardCharsets.UTF_8)) {

      for (entriesLoaded = 0;
          entriesLoaded < MAX_LOOKUP_TABLE_SIZE && scanner.hasNextLine();
          entriesLoaded++) {
        String[] entry = scanner.nextLine().split(DELIMITER);

        double distance = Double.parseDouble(entry[DISTANCE]);
        double error = Double.parseDouble(entry[ERROR]);
        totalError += error;

        speedLookup.put(distance, Double.parseDouble(entry[SPEED]));
        pitchLookup.put(distance, Double.parseDouble(entry[PITCH]));
        errorLookup.put(distance, error);
      }

      averageError = entriesLoaded == 0 ? 0 : totalError / entriesLoaded;
      status = entriesLoaded > 0;

    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * The speed interpolated from the lookup table for the given distance.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   */
  public static double speed(double distance) {
    return status() ? speedLookup.get(distance) : MIN_SPEED;
  }

  /**
   * The pitch interpolated from the lookup table for the given distance.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   */
  public static double pitch(double distance) {
    return status() ? pitchLookup.get(distance) : MIN_PITCH;
  }

  /**
   * The error interpolated from the lookup table for the given distance.
   *
   * @param distance the planar distance of the shooter from the HUB in meters
   */
  public static double error(double distance) {
    return status() ? errorLookup.get(distance) : 0;
  }

  /** Whether or not a table has been loaded into the interpolators. */
  public static boolean status() {
    return status;
  }

  /** The average planar error of all the lookup table entries. */
  public static double averageError() {
    return averageError;
  }

  /** Logs table data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/Parameter Lookup/Status", status);
    LoggingUtils.log("Shooting/Parameter Lookup/Average Error", averageError);
    LoggingUtils.log("Shooting/Parameter Lookup/Entries Generated", entriesGenerated);
    LoggingUtils.log("Shooting/Parameter Lookup/Generation Complete", generationComplete);
    LoggingUtils.log("Shooting/Parameter Lookup/Entries Loaded", entriesLoaded);
  }
}
