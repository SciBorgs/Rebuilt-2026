package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShotOptimizer.MAXIMUM_ANGLE;
import static org.sciborgs1155.robot.commands.shooting.ShotOptimizer.MINIMUM_ANGLE;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Scanner;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;

public final class TableGenerator {
  private static final double MIN_DISTANCE = 0.1;
  private static final double MAX_DISTANCE = 15;

  private static final double INCREMENT = 0.01;
  private static final String NAME = "LookUpTable";

  private static final double SPEED_DEADBAND = 0.01;

  private static final InterpolatingDoubleTreeMap SPEED_TABLE = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap ANGLE_TABLE = new InterpolatingDoubleTreeMap();

  private static final int MAX_TABLE_SIZE = 1000;

  private TableGenerator() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command createTable() {
    return Commands.runOnce(() -> createTable(MIN_DISTANCE, MAX_DISTANCE, INCREMENT, NAME));
  }

  static void createTable(
      double minDistance, double maxDistance, double increment, String tableName) {
    try {
      Tracer.startTrace("lookup table generation");
      LoggingUtils.log("Shooting/Entries Generated", 0);
      BufferedWriter fileWriter =
          Files.newBufferedWriter(
              Paths.get("resources/" + tableName + ".txt"), StandardCharsets.UTF_8);

      int tableIndex = 0;
      for (double distance = minDistance; distance < maxDistance; distance += increment) {
        double[] launchParameters = ShotOptimizer.directLaunchParameters(distance);

        double speed = launchParameters[SPEED];
        double angle = launchParameters[PITCH];

        if (speed < SPEED_DEADBAND) continue;
        if (angle < MINIMUM_ANGLE) continue;
        if (angle > MAXIMUM_ANGLE) continue;

        // FORMAT: [DISTANCE]/[SPEED]/[ANGLE](SPACE)
        fileWriter.write(distance + "," + speed + "," + angle + " ");
        tableIndex++;
        LoggingUtils.log("Shooting/Entries Generated", tableIndex);
      }

      fileWriter.close();
      Tracer.endTrace();
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
    return Commands.runOnce(() -> loadTable(NAME));
  }

  static void loadTable(String tableName) {
    SPEED_TABLE.clear();
    ANGLE_TABLE.clear();

    try {
      Tracer.startTrace("lookup table loading");
      LoggingUtils.log("Shooting/Entries Loaded", 0);
      Scanner fileScanner =
          new Scanner(new File("resources/" + tableName + ".txt"), StandardCharsets.UTF_8);

      int tableIndex = 0;
      while (tableIndex < MAX_TABLE_SIZE && fileScanner.hasNext()) {
        String entry = fileScanner.next();

        int comma1Index = entry.indexOf(',');
        int comma2Index = entry.indexOf(',', comma1Index + 1);

        double distance = Double.parseDouble(entry.substring(0, comma1Index));
        double speed = Double.parseDouble(entry.substring(comma1Index + 1, comma2Index));
        double angle = Double.parseDouble(entry.substring(comma2Index + 1));

        SPEED_TABLE.put(distance, speed);
        ANGLE_TABLE.put(distance, angle);

        tableIndex++;
        LoggingUtils.log("Shooting/Entries Loaded", tableIndex);
      }

      fileScanner.close();
      Tracer.endTrace();
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  static double[] directLaunchParameters(double distance) {
    try {
      LoggingUtils.log("Shooting/LookUp Table Status", true);
      return new double[] {distance, SPEED_TABLE.get(distance), ANGLE_TABLE.get(distance), 0};
    } catch (Exception e) {
      LoggingUtils.log("Shooting/LookUp Table Status", false);
      return new double[] {distance, 0, 0, 0};
    }
  }
}
