package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;
import org.sciborgs1155.lib.LoggingUtils;

/**
 * A utility class used to generate a distance/speed/pitch lookup table which can be referenced when
 * calculating launch parameters for shooting.
 */
@SuppressWarnings("PMD.FieldNamingConventions")
public final class ShotLookUpTable {
  private static final InterpolatingDoubleTreeMap speedLookUp = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap pitchLookUp = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap errorLookUp = new InterpolatingDoubleTreeMap();

  private static boolean status;

  private ShotLookUpTable() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command generate() {
    return Commands.runOnce(() -> generateTable(MIN_DISTANCE, MAX_DISTANCE, INCREMENT, PATH));
  }

  private static void generateTable(
      double minDistance, double maxDistance, double increment, String tablePath) {
    try {
      LoggingUtils.log("Shooting/Entries Generated", 0);

      Path path = Paths.get("resources/" + tablePath + ".ankit");
      BufferedWriter fileWriter = Files.newBufferedWriter(path, StandardCharsets.UTF_8);

      int tableIndex = 0;
      double totalError = 0;
      for (double distance = minDistance; distance < maxDistance; distance += increment) {
        double[] launchParameters = ShotOptimizer.optimizedLaunchParameters(distance);

        double speed = launchParameters[SPEED];
        double pitch = launchParameters[PITCH];
        double error = launchParameters[ERROR];

        if (speed > MAX_SPEED) continue;
        if (speed < SPEED_DEADBAND) continue;

        if (pitch < MIN_PITCH) continue;
        if (pitch > MAX_PITCH) continue;

        double averageError = totalError / tableIndex;
        if (Math.abs(error - averageError) > ERROR_THRESHOLD) continue;

        totalError += error;
        fileWriter.write(distance + "," + speed + "," + pitch + "," + error);
        fileWriter.newLine();

        tableIndex++;
        LoggingUtils.log("Shooting/Entries Generated", tableIndex);
      }

      fileWriter.close();
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
    return Commands.runOnce(() -> loadTable(PATH));
  }

  private static void loadTable(String tablePath) {
    speedLookUp.clear();
    pitchLookUp.clear();
    errorLookUp.clear();

    try {
      LoggingUtils.log("Shooting/Entries Loaded", 0);
      Scanner fileScanner =
          new Scanner(new File("resources/" + tablePath + ".ankit"), StandardCharsets.UTF_8);

      int tableIndex = 0;
      double totalError = 0;
      while (tableIndex < MAX_TABLE_SIZE && fileScanner.hasNextLine()) {
        String entry = fileScanner.nextLine();

        int comma1Index = entry.indexOf(',');
        int comma2Index = entry.indexOf(',', comma1Index + 1);
        int comma3Index = entry.indexOf(',', comma2Index + 1);

        double distance = Double.parseDouble(entry.substring(0, comma1Index));
        double speed = Double.parseDouble(entry.substring(comma1Index + 1, comma2Index));
        double pitch = Double.parseDouble(entry.substring(comma2Index + 1, comma3Index));
        double error = Double.parseDouble(entry.substring(comma3Index + 1));
        totalError += error;

        speedLookUp.put(distance, speed);
        pitchLookUp.put(distance, pitch);
        errorLookUp.put(distance, error);

        tableIndex++;
        LoggingUtils.log("Shooting/Entries Loaded", tableIndex);
      }

      LoggingUtils.log("Shooting/Average Error", totalError / tableIndex);
      fileScanner.close();
      if (tableIndex > 0) status = true;
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /** The speed returned from the lookup table for the given distance. */
  public static double speed(double distance) {
    if (!status()) return 0;
    LoggingUtils.log("Shooting/LookUp Table Status", status);
    return speedLookUp.get(distance);
  }

  /** The pitch returned from the lookup table for the given distance. */
  public static double pitch(double distance) {
    if (!status()) return MIN_PITCH;
    LoggingUtils.log("Shooting/LookUp Table Status", status);
    return pitchLookUp.get(distance);
  }

  /**
   * Whether or not a lookup table has been loaded (calls to 'speed' and 'pitch' require a loaded
   * lookup table).
   */
  public static boolean status() {
    return status;
  }
}
