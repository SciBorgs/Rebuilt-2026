package org.sciborgs1155.robot.commands.shooting;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class ShotTable {
  private static boolean loaded = false;

  private static InterpolatingDoubleTreeMap speedTable = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap angleTable = new InterpolatingDoubleTreeMap();

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
      FileWriter fileWriter = new FileWriter(new File("resources/" + name + ".txt"));

      for (double distance = minDistance; distance < maxDistance; distance += increment) {
        double[] launchParameters = ShotOptimizer.distanceSpeedAndPitch(distance);

        // FORMAT: [DISTANCE]/[SPEED]/[ANGLE](SPACE)
        fileWriter.write(
            distance
                + ","
                + launchParameters[ShotOptimizer.SPEED]
                + ","
                + launchParameters[ShotOptimizer.PITCH]
                + " ");
      }

      fileWriter.close();
    } catch (IOException exception) {
      System.out.println("Failed to generate shot table! ");
      exception.printStackTrace();
    }
  }

  public static void loadShotTable(String shotTableName) {
    speedTable.clear();
    angleTable.clear();

    try {
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

        if (speed < 0.0000000001) continue;

        speedTable.put(distance, speed);
        angleTable.put(distance, angle);

        treeIndex++;
      }

      fileScanner.close();
      loaded = true;
    } catch (Exception exception) {
      System.out.println("Failed to load shot table!");
      exception.printStackTrace();
    }
  }

  public static double speed(Pose3d robotPose) {
    return loaded ? speedTable.get(FuelVisualizer.distanceToHub(robotPose)) : 0;
  }

  public static double angle(Pose3d robotPose) {
    return loaded ? angleTable.get(FuelVisualizer.distanceToHub(robotPose)) : 0;
  }
}
