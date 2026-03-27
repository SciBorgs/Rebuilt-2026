package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DISTANCE_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ERROR;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_ERROR;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_LOOKUP_TABLE_SIZE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MIN_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PARAMETER_TABLE_PATH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_ERROR;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_SPEED;

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

/**
 * A utility class used to generate a distance/speed/pitch/error lookup table which can be
 * referenced when calculating launch parameters for shooting.
 */
@SuppressWarnings({"PMD.OneDeclarationPerLine", "PMD.CyclomaticComplexity"})
public final class ParameterLookup {
  private static InterpolatingDoubleTreeMap speedLookup = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap pitchLookup = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap errorLookup = new InterpolatingDoubleTreeMap();

  private static boolean status;
  private static double entriesGenerated;
  private static double entriesLoaded, averageError;

  private static ScheduledExecutorService executor =
      Executors.newScheduledThreadPool(
          1,
          runnable -> {
            Thread thread = new Thread(runnable);
            thread.setName("ShotLookupTable Generator");
            return thread;
          });

  private ParameterLookup() {}

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
                        PARAMETER_TABLE_PATH, MIN_DISTANCE, MAX_DISTANCE, DISTANCE_RESOLUTION)));
  }

  private static void generateTable(String name, double min, double max, double resolution) {
    entriesGenerated = 0;
    double totalError = 0;
    double increment = 1 / resolution;

    ShotOptimizer.clearCache();
    Path path = Path.of("resources/shooting/%s.ankit".formatted(name));

    try (BufferedWriter writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8)) {
      for (double distance = max;
          distance >= min && entriesGenerated <= MAX_LOOKUP_TABLE_SIZE;
          distance -= increment) {
        double[] entry = ShotOptimizer.optimizeForAirTime(distance);

        double speed = entry[SPEED];
        double pitch = entry[PITCH];

        if (speed > MAX_SPEED || speed < MIN_SPEED || pitch < MIN_PITCH || pitch > MAX_PITCH)
          continue;

        double error = entry[ERROR];
        if (entriesGenerated > 0 && Math.abs(error - totalError / entriesGenerated) > MAX_ERROR)
          continue;

        totalError += error;
        writer.write("%.4f,%.10f,%.10f,%.10f".formatted(distance, speed, pitch, error));
        writer.newLine();

        entriesGenerated++;
      }
    } catch (IOException exception) {
      exception.printStackTrace();
    }
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
    Path path = Path.of("resources/shooting/%s.ankit".formatted(name));

    try (Scanner scanner = new Scanner(path, StandardCharsets.UTF_8)) {

      for (entriesLoaded = 0;
          entriesLoaded < MAX_LOOKUP_TABLE_SIZE && scanner.hasNextLine();
          entriesLoaded++) {
        String[] entry = scanner.nextLine().split(",");

        double distance = Double.parseDouble(entry[TABLE_DISTANCE]);
        double error = Double.parseDouble(entry[TABLE_ERROR]);
        totalError += error;

        speedLookup.put(distance, Double.parseDouble(entry[TABLE_SPEED]));
        pitchLookup.put(distance, Double.parseDouble(entry[TABLE_PITCH]));
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

  /** Whether or not a table has been loaded into the interpolators. */
  public static boolean status() {
    return status;
  }

  /** The average planar distance from the target of all lookup table entries. */
  public static double error() {
    return averageError;
  }

  /** Logs table data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/ParameterLookup/Status", status);
    LoggingUtils.log("Shooting/ParameterLookup/Average Error", averageError);
    LoggingUtils.log("Shooting/ParameterLookup/Entries Generated", entriesGenerated);
    LoggingUtils.log("Shooting/ParameterLookup/Entries Loaded", entriesLoaded);
  }
}
