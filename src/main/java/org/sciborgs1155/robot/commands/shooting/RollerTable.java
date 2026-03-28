package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.toPitch;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.BufferedWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Scanner;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.DistanceTableConstants;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.RollerTableConstants;

/** A utility class used to analyze shooting velocity data and compile it into a look up table. */
public final class RollerTable {
  private static InterpolatingDoubleTreeMap rollerSpeedLookup = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap speedLookup = new InterpolatingDoubleTreeMap();

  private static boolean status;
  private static double averageError;

  private static int entriesGenerated;
  private static int entriesLoaded;

  private RollerTable() {}

  /**
   * Generates a new lookup table.
   *
   * @return a command to generate the lookup table
   */
  public static Command generate() {
    return Commands.runOnce(
        () -> generateTable(RollerTableConstants.TABLE_PATH, DistanceTableConstants.TABLE_PATH));
  }

  private static void generateTable(String name, String standardTableName) {
    Path rollerTablePath = Path.of("resources/shooting/%s.ankit".formatted(name));
    Path distanceTablePath = Path.of("resources/shooting/%s.ankit".formatted(standardTableName));

    try (BufferedWriter writer = Files.newBufferedWriter(rollerTablePath, StandardCharsets.UTF_8);
        Scanner scanner = new Scanner(distanceTablePath, StandardCharsets.UTF_8); ) {
      entriesGenerated = 0;
      while (scanner.hasNextLine()) {
        String[] entry = scanner.nextLine().split(",");

        double distance = Double.parseDouble(entry[DistanceTableConstants.DISTANCE]);
        double rollerSpeed = Double.parseDouble(entry[DistanceTableConstants.ROLLER_SPEED]);
        double hoodAngle =
            Math.toRadians(Double.parseDouble(entry[DistanceTableConstants.HOOD_ANGLE]));

        double pitch = toPitch(hoodAngle);
        double timeOfFlight = Double.parseDouble(entry[DistanceTableConstants.TIME_OF_FLIGHT]);
        double speed = ShotOptimizer.estimateSpeed(distance, pitch, timeOfFlight);
        double actualTimeOfFlight =
            ShotOptimizer.timeOfFlight(distance, new double[] {speed, pitch, 0});
        double error = Math.abs(actualTimeOfFlight - timeOfFlight);

        entriesGenerated++;
        writer.write(RollerTableConstants.FORMAT.formatted(speed, rollerSpeed, error));
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
    return Commands.runOnce(() -> loadTable(RollerTableConstants.TABLE_PATH));
  }

  private static void loadTable(String name) {
    rollerSpeedLookup.clear();
    speedLookup.clear();

    Path path = Path.of(Filesystem.getDeployDirectory() + "/shooting/%s.ankit".formatted(name));

    status = false;
    entriesLoaded = 0;
    try (Scanner scanner = new Scanner(path, StandardCharsets.UTF_8)) {
      double totalError = 0;

      while (scanner.hasNextLine()) {
        String[] entry = scanner.nextLine().split(RollerTableConstants.DELIMITER);
        double speed = Double.parseDouble(entry[RollerTableConstants.LAUNCH_SPEED]);
        double rollerSpeed = Double.parseDouble(entry[RollerTableConstants.ROLLER_SPEED]);
        double error = Double.parseDouble(entry[RollerTableConstants.ERROR]);
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
