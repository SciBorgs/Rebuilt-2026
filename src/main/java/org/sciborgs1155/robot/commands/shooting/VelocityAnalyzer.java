package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OPTIMIZER_RESOLUTION;

import java.io.BufferedWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import org.sciborgs1155.lib.LoggingUtils;

/** A utility class used to analyze shooting velocity data and compile it into a look up table. */
public final class VelocityAnalyzer {
  private VelocityAnalyzer() {}

  public static double calculateSpeed(double distance, double pitchDegrees, double timeOfFlight) {
    double pitch = Math.PI / 2 - Units.degreesToRadians(pitchDegrees);
    double speed = ShotOptimizer.estimateSpeed(distance, pitch, timeOfFlight);

    double[] launchParameters = {speed, pitch, 0};
    ShotOptimizer.generateDirectTrajectory(distance, launchParameters);

    double[][] trajectory = ShotOptimizer.buffer();
    Pose3d[] poses = new Pose3d[trajectory.length];

    for (int frame = 0; frame < trajectory.length; frame++) {
      double[] translation = trajectory[frame];
      poses[frame] = new Pose3d(translation[X], translation[Y], translation[Z], new Rotation3d());
    }

    double simulatedTOF = trajectory.length * 1.0 / OPTIMIZER_RESOLUTION;
    double error = (simulatedTOF - timeOfFlight) / simulatedTOF;

    LoggingUtils.log("Shooting/Velocity Analyzer/Time of Flight", simulatedTOF);
    LoggingUtils.log("Shooting/Velocity Analyzer/Error", error);
    LoggingUtils.log("Shooting/Velocity Analyzer/Estimated Speed", speed);
    LoggingUtils.log("Shooting/Velocity Analyzer/Recreation", poses, Pose3d.struct);

    return speed;
  }

  // public static void generateTable(String name) {
  //   Path path = Path.of("resources/shooting/%s.ankit".formatted(name));

  //   try (BufferedWriter writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8)) {
      

  //     writer.write("%.4f,%.10f,%.10f,%.10f".formatted(speed, rpm));
  //     writer.newLine();
  //   }
  // }


}
