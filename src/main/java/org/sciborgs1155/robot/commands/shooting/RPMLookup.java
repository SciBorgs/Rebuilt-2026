package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.RPM_TABLE_PATH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.STANDARD_TABLE_PATH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TABLE_TOF;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.BufferedWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Scanner;
import org.sciborgs1155.lib.LoggingUtils;

/** A utility class used to analyze shooting velocity data and compile it into a look up table. */
public final class RPMLookup {
  private static InterpolatingDoubleTreeMap rpmLookup = new InterpolatingDoubleTreeMap();

  private static boolean status;
  private static double entriesGenerated, entriesLoaded;

  private RPMLookup() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command generate() {
    return Commands.runOnce(() -> generateTable(RPM_TABLE_PATH, STANDARD_TABLE_PATH));
  }

  private static void generateTable(String name, String standardTableName) {
    try {
      Path rpmPath = Path.of("resources/shooting/%s.ankit".formatted(name));
      Path standardPath = Path.of("resources/shooting/%s.ankit".formatted(standardTableName));

      BufferedWriter writer = Files.newBufferedWriter(rpmPath, StandardCharsets.UTF_8);
      Scanner scanner = new Scanner(standardPath, StandardCharsets.UTF_8);

      entriesGenerated = 0;
      while (scanner.hasNextLine()) {
        String[] entry = scanner.nextLine().split(",");

        double distance = Double.parseDouble(entry[TABLE_DISTANCE]);
        double rpm = Double.parseDouble(entry[TABLE_SPEED]);
        double hoodAngle = Math.toRadians(Double.parseDouble(entry[TABLE_PITCH]));
        double timeOfFlight = Double.parseDouble(entry[TABLE_TOF]);
        double speed = ShotOptimizer.estimateSpeed(distance, Math.PI / 2 - hoodAngle, timeOfFlight);

        entriesGenerated++;
        writer.write("%.4f,%.4f".formatted(speed, rpm));
        writer.newLine();
      }

      writer.close();
      scanner.close();
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * Loads the lookup table from the resources folder.
   *
   * @return a command to load the lookup table
   */
  public static Command load() {
    return Commands.runOnce(() -> loadTable(RPM_TABLE_PATH));
  }

  private static void loadTable(String name) {
    Path path = Path.of("resources/shooting/%s.ankit".formatted(name));

    status = false;
    entriesLoaded = 0;
    try (Scanner scanner = new Scanner(path, StandardCharsets.UTF_8)) {
      while (scanner.hasNextLine()) {
        String[] entry = scanner.nextLine().split(",");
        double speed = Double.parseDouble(entry[TABLE_DISTANCE]);
        double rpm = Double.parseDouble(entry[TABLE_SPEED]);
        rpmLookup.put(speed, rpm);
        entriesLoaded++;
      }

      status = true;
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * The rpm interpolated from the lookup table for the given launch speed.
   *
   * @param speed target launch speed in meters per second
   */
  public static double rpm(double speed) {
    return status() ? rpmLookup.get(speed) : 0.0;
  }

  /** Whether or not a table has been loaded into the interpolator. */
  public static boolean status() {
    return status;
  }

  /** Logs table data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/RPMLookup/Status", status);
    LoggingUtils.log("Shooting/RPMLookup/Entries Generated", entriesGenerated);
    LoggingUtils.log("Shooting/RPMLookup/Entries Loaded", entriesLoaded);
  }
}
