package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShotGenerator.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShotGenerator.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShotGenerator.MIN_PITCH;

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
import org.sciborgs1155.robot.FieldConstants.Hub;

public final class TableGenerator {
  private static final double MIN_DISTANCE = Hub.WIDTH / 2;
  private static final double MAX_DISTANCE = 20;

  private static final double INCREMENT = 0.005;
  private static final String PATH = "shooting/ParameterLookUp";

  private static final double SPEED_DEADBAND = 0.01;
  private static final double ERROR_THRESHOLD = 1.5;

  static final InterpolatingDoubleTreeMap SPEED_TABLE = new InterpolatingDoubleTreeMap();
  static final InterpolatingDoubleTreeMap PITCH_TABLE = new InterpolatingDoubleTreeMap();
  static final InterpolatingDoubleTreeMap ERROR_TABLE = new InterpolatingDoubleTreeMap();

  private static boolean loaded = false;

  private static final int MAX_TABLE_SIZE = 5000;
  private static final int ERROR = 3;

  private TableGenerator() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command createTable() {
    return Commands.runOnce(() -> createTable(MIN_DISTANCE, MAX_DISTANCE, INCREMENT, PATH));
  }

  private static void createTable(
      double minDistance, double maxDistance, double increment, String tablePath) {
    try {
      LoggingUtils.log("Shooting/Entries Generated", 0);

      Path path = Paths.get("resources/" + tablePath + ".ankit");
      BufferedWriter fileWriter = Files.newBufferedWriter(path, StandardCharsets.UTF_8);

      int tableIndex = 0;
      double totalError = 0;
      for (double distance = minDistance; distance < maxDistance; distance += increment) {
        double[] launchParameters = ShotGenerator.optimizedLaunchParameters(distance);

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
  public static Command loadTable() {
    return Commands.runOnce(() -> loadTable(PATH));
  }

  private static void loadTable(String tablePath) {
    SPEED_TABLE.clear();
    PITCH_TABLE.clear();
    ERROR_TABLE.clear();

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

        SPEED_TABLE.put(distance, speed);
        PITCH_TABLE.put(distance, pitch);
        ERROR_TABLE.put(distance, error);

        tableIndex++;
        LoggingUtils.log("Shooting/Entries Loaded", tableIndex);
      }

      LoggingUtils.log("Shooting/Average Error", totalError / tableIndex);
      fileScanner.close();
      if (tableIndex > 0) loaded = true;
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * Returns the launch parameters (speed, pitch, yaw) required to shoot from a certain planar
   * distance into the hub. Distance is to the origin of the shooter.
   */
  public static double[] directLaunchParameters(double distance) {
    LoggingUtils.log("Shooting/LookUp Table Status", loaded);

    if (loaded) return new double[] {SPEED_TABLE.get(distance), PITCH_TABLE.get(distance), 0};
    else return new double[] {0, 0, 0};
  }

  static boolean loaded() {
    return loaded;
  }
}
