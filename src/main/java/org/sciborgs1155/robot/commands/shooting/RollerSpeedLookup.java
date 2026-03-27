package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DISTANCE_TABLE_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DISTANCE_TABLE_PATH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DISTANCE_TABLE_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DISTANCE_TABLE_ROLLER_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DISTANCE_TABLE_TOF;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ROLLER_TABLE_ERROR;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ROLLER_TABLE_PATH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ROLLER_TABLE_ROLLER_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ROLLER_TABLE_SPEED;

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
public final class RollerSpeedLookup {
  private static InterpolatingDoubleTreeMap rollerSpeedLookup = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap speedLookup = new InterpolatingDoubleTreeMap();

  private static boolean status;
  private static int entriesGenerated, entriesLoaded;
  private static double averageError;

  private RollerSpeedLookup() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command generate() {
    return Commands.runOnce(() -> generateTable(ROLLER_TABLE_PATH, DISTANCE_TABLE_PATH));
  }

  private static void generateTable(String name, String standardTableName) {
    Path rollerTablePath = Path.of("resources/shooting/%s.ankit".formatted(name));
    Path distanceTablePath = Path.of("resources/shooting/%s.ankit".formatted(standardTableName));

    try (BufferedWriter writer = Files.newBufferedWriter(rollerTablePath, StandardCharsets.UTF_8);
        Scanner scanner = new Scanner(distanceTablePath, StandardCharsets.UTF_8); ) {
      entriesGenerated = 0;
      while (scanner.hasNextLine()) {
        String[] entry = scanner.nextLine().split(",");

        double distance = Double.parseDouble(entry[DISTANCE_TABLE_DISTANCE]);
        double rollerSpeed = Double.parseDouble(entry[DISTANCE_TABLE_ROLLER_SPEED]);
        double hoodAngle = Math.toRadians(Double.parseDouble(entry[DISTANCE_TABLE_PITCH]));
        double pitch = Math.PI / 2 - hoodAngle;
        double timeOfFlight = Double.parseDouble(entry[DISTANCE_TABLE_TOF]);
        double speed = ShotOptimizer.estimateSpeed(distance, pitch, timeOfFlight);
        double actualTimeOfFlight =
            ShotOptimizer.timeOfFlight(distance, new double[] {speed, pitch, 0});
        double error = Math.abs(actualTimeOfFlight - timeOfFlight);

        entriesGenerated++;
        writer.write("%.4f,%.4f,%.10f".formatted(speed, rollerSpeed, error));
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
    return Commands.runOnce(() -> loadTable(ROLLER_TABLE_PATH));
  }

  private static void loadTable(String name) {
    rollerSpeedLookup.clear();
    speedLookup.clear();

    Path path = Path.of("resources/shooting/%s.ankit".formatted(name));

    status = false;
    entriesLoaded = 0;
    try (Scanner scanner = new Scanner(path, StandardCharsets.UTF_8)) {
      double totalError = 0;

      while (scanner.hasNextLine()) {
        String[] entry = scanner.nextLine().split(",");
        double speed = Double.parseDouble(entry[ROLLER_TABLE_SPEED]);
        double rollerSpeed = Double.parseDouble(entry[ROLLER_TABLE_ROLLER_SPEED]);

        double error = Double.parseDouble(entry[ROLLER_TABLE_ERROR]);
        totalError += error;

        rollerSpeedLookup.put(speed, rollerSpeed);
        speedLookup.put(rollerSpeed, speed);
        entriesLoaded++;
      }

      averageError = entriesLoaded == 0 ? 0 : totalError / entriesLoaded;
      status = true;
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * The roller speed interpolated from the lookup table for the given launch speed, in radians per
   * second.
   *
   * @param speed target launch speed in meters per second
   */
  public static double rollerSpeed(double speed) {
    return status() ? rollerSpeedLookup.get(speed) : 0.0;
  }

  /**
   * The launch speed interpolated from the lookup table for the given roller speed, in meters per
   * second.
   *
   * @param rollerSpeed angular velocity of the shooter rollers in radians per second
   */
  public static double speed(double rollerSpeed) {
    return status() ? speedLookup.get(rollerSpeed) : 0.0;
  }

  /** Whether or not a table has been loaded into the interpolator. */
  public static boolean status() {
    return status;
  }

  /** Logs table data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/Roller Speed Lookup/Status", status);
    LoggingUtils.log("Shooting/Roller Speed Lookup/Average Error", averageError);
    LoggingUtils.log("Shooting/Roller Speed Lookup/Entries Generated", entriesGenerated);
    LoggingUtils.log("Shooting/Roller Speed Lookup/Entries Loaded", entriesLoaded);
  }
}
