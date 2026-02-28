package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShotOptimizer.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import org.sciborgs1155.lib.Tracer;

public class ShotTable {
  private static boolean loaded = false;

  private static final InterpolatingDoubleTreeMap SPEED_TABLE = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap ANGLE_TABLE = new InterpolatingDoubleTreeMap();

  private static final int MAX_TREE_LENGTH = 1000;

  public static Command createShotTableCommand(
      double minDistance, double maxDistance, double increment, String name) {
    return Commands.runOnce(() -> createShotTable(minDistance, maxDistance, increment, name));
  }

  public static Command loadShotTableCommand(String shotTableName) {
    return Commands.runOnce(() -> loadShotTable(shotTableName));
  }

  public static void createShotTable(
      double minDistance, double maxDistance, double increment, String name) {
    try {
      Tracer.startTrace("shot table generation");
      FileWriter fileWriter = new FileWriter(new File("resources/" + name + ".txt"));

      for (double distance = minDistance; distance < maxDistance; distance += increment) {
        double[] launchParameters = distanceSpeedAndPitch(distance);

        double speed = launchParameters[SPEED];
        double angle = launchParameters[PITCH];

        if (speed < 0.01) continue;

        // FORMAT: [DISTANCE]/[SPEED]/[ANGLE](SPACE)
        fileWriter.write(distance + "," + speed + "," + angle + " ");
      }

      fileWriter.close();
      System.out.println("Successfully generated shot table!");
      Tracer.endTrace();
    } catch (IOException exception) {
      System.out.println("Failed to generate shot table! ");
      exception.printStackTrace();
    }
  }

  public static void loadShotTable(String shotTableName) {
    SPEED_TABLE.clear();
    ANGLE_TABLE.clear();

    try {
      Tracer.startTrace("shot table generation");
      Scanner fileScanner = new Scanner(new File("resources/" + shotTableName + ".txt"));

      int treeIndex = 0;
      while (treeIndex < MAX_TREE_LENGTH && fileScanner.hasNext()) {
        String entry = fileScanner.next();
        if (entry.length() < 5) continue;

        int comma1Index = entry.indexOf(",");
        int comma2Index = entry.indexOf(",", comma1Index + 1);

        double distance = Double.parseDouble(entry.substring(0, comma1Index));
        double speed = Double.parseDouble(entry.substring(comma1Index + 1, comma2Index));
        double angle = Double.parseDouble(entry.substring(comma2Index + 1));

        SPEED_TABLE.put(distance, speed);
        ANGLE_TABLE.put(distance, angle);

        treeIndex++;
      }

      fileScanner.close();
      loaded = true;
      System.out.println("Successfully loaded shot table!");
      Tracer.endTrace();
    } catch (Exception exception) {
      System.out.println("Failed to load shot table!");
      exception.printStackTrace();
    }
  }

  public static double speed(double distance) {
    return loaded ? SPEED_TABLE.get(distance) : 0;
  }

  public static double angle(double distance) {
    return loaded ? ANGLE_TABLE.get(distance) : 0;
  }
}
