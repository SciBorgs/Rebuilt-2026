package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterTableConstants.*;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.*;
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

/**
 * Generates and stores a distance → pitch/speed lookup table for shooting parameter interpolation.
 */
public final class ParameterTable {

  /** Represents a single row in the parameter lookup table. */
  public record ShotData(
      double distance, double speed, double pitch, double error, double timeOfFlight) {

    public static ShotData fromArray(String[] entry) {
      if (entry.length != 5)
        throw new IllegalArgumentException("Invalid entry: expected 5 fields, got " + entry.length);
      return new ShotData(
          Double.parseDouble(entry[0]),
          Double.parseDouble(entry[1]),
          Double.parseDouble(entry[2]),
          Double.parseDouble(entry[3]),
          Double.parseDouble(entry[4]));
    }

    public boolean isOutOfBounds() {
      return distance < MIN_DISTANCE
          || distance > MAX_DISTANCE
          || speed < MIN_SPEED
          || speed > MAX_SPEED
          || pitch < MIN_PITCH
          || pitch > MAX_PITCH
          || error > MAX_ERROR;
    }
  }

  private static final InterpolatingDoubleTreeMap speedLookup = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap pitchLookup = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap errorLookup = new InterpolatingDoubleTreeMap();

  private static final ScheduledExecutorService executor =
      Executors.newScheduledThreadPool(
          1, runnable -> new Thread(runnable, "Parameter Table Generation"));

  private static boolean loadComplete;
  private static boolean generationComplete;
  private static double averageError;
  private static int entriesGenerated;
  private static int entriesLoaded;

  private ParameterTable() {}

  private static Path tablePath() {
    return Path.of(TABLE_DIRECTORY + "%s.ankit".formatted(PARAMETER_TABLE_PATH));
  }

  /**
   * @return a command that asynchronously generates and writes the lookup table to disk.
   */
  public static Command generate() {
    return Commands.runOnce(ParameterTable::generateTable)
        .andThen(Commands.idle())
        .until(ParameterTable::generationStatus);
  }

  private static void generateTable() {
    generationComplete = false;
    entriesGenerated = 0;
    ShotOptimizer.clearCache();

    executor.submit(() -> {
      try(BufferedWriter writer = Files.newBufferedWriter(tablePath(), StandardCharsets.UTF_8)) {
        for (double distance = MAX_DISTANCE; distance >= MIN_DISTANCE; distance -= 1.0 / DISTANCE_RESOLUTION) {
        if (entriesGenerated >= MAX_LOOKUP_TABLE_SIZE) break;
          ShotData entry = ShotOptimizer.optimizeForAirTime(distance);
          if (entry.isOutOfBounds()) continue;
          writer.write(
              FORMAT.formatted(
                  entry.distance(),
                  entry.speed(),
                  entry.pitch(),
                  entry.error(),
                  entry.timeOfFlight()));
          writer.newLine();
          entriesGenerated++;
        }

        writer.close();
        generationComplete = true;
      } catch (IOException exception) {
        exception.printStackTrace();
      }
    });
  }

  /**
   * @return a command that loads the lookup table from disk into the interpolators.
   */
  public static Command load() {
    return Commands.runOnce(ParameterTable::loadTable)
        .andThen(Commands.idle())
        .until(ParameterTable::loadStatus);
  }

  private static void loadTable() {
    loadComplete = false;

    speedLookup.clear();
    pitchLookup.clear();
    errorLookup.clear();

    double totalError = 0;
    try (Scanner scanner = new Scanner(tablePath(), StandardCharsets.UTF_8)) {
      for (entriesLoaded = 0;
          entriesLoaded < MAX_LOOKUP_TABLE_SIZE && scanner.hasNextLine();
          entriesLoaded++) {
        ShotData entry = ShotData.fromArray(scanner.nextLine().split(DELIMITER));
        totalError += entry.error();
        speedLookup.put(entry.distance(), entry.speed());
        pitchLookup.put(entry.distance(), entry.pitch());
        errorLookup.put(entry.distance(), entry.error());

      scanner.close();
      averageError = entriesLoaded == 0 ? 0 : totalError / entriesLoaded;
      loadComplete = true;
    }
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * @param distance planar distance from the shooter to the HUB in meters
   */
  public static double speed(double distance) {
    return loadStatus() ? speedLookup.get(distance) : MIN_SPEED;
  }

  /**
   * @param distance planar distance from the shooter to the HUB in meters
   */
  public static double pitch(double distance) {
    return loadStatus() ? pitchLookup.get(distance) : MIN_PITCH;
  }

  /**
   * @param distance planar distance from the shooter to the HUB in meters
   */
  public static double error(double distance) {
    return loadStatus() ? errorLookup.get(distance) : 0;
  }

  /**
   * @return whether a table has been successfully loaded into the interpolators
   */
  public static boolean loadStatus() {
    return loadComplete;
  }

  /**
   * @return whether the most recent table has been successfully generated
   */
  public static boolean generationStatus() {
    return generationComplete;
  }

  /**
   * @return average planar error across all loaded lookup table entries
   */
  public static double averageError() {
    return averageError;
  }

  public static void updateLogging() {
    LoggingUtils.log("Shooting/Parameter Lookup/Load Status", loadComplete);
    LoggingUtils.log("Shooting/Parameter Lookup/Generation Status", generationComplete);
    LoggingUtils.log("Shooting/Parameter Lookup/Average Error", averageError);
    LoggingUtils.log("Shooting/Parameter Lookup/Entries Generated", entriesGenerated);
    LoggingUtils.log("Shooting/Parameter Lookup/Entries Loaded", entriesLoaded);
  }
}
