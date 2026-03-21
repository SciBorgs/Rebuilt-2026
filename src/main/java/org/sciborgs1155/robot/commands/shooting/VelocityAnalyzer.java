package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.OPTIMIZER_RESOLUTION;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.LoggingUtils;

public final class VelocityAnalyzer {
  private VelocityAnalyzer() {}

  /** Javadoc. */
  public static Command tableErrorAnalysis() {
    return Commands.runOnce(() -> tofError(2.44, Units.degreesToRadians(70), 1.0));
  }

  private static double tofError(double distance, double pitch, double tof) {
    double speed = ShotOptimizer.getSpeed(distance, pitch, tof);
    double[] launchParameters = {speed, pitch, 0};
    ShotOptimizer.generateDirectTrajectory(distance, launchParameters);
    double[][] trajectory = ShotOptimizer.buffer();
    Pose3d[] poses = new Pose3d[trajectory.length];

    for (int frame = 0; frame < trajectory.length; frame++) {
      double[] translation = trajectory[frame];
      poses[frame] = new Pose3d(translation[X], translation[Y], translation[Z], new Rotation3d());
    }

    LoggingUtils.log("Shooting/Velocity Analyzer/Recreation", poses, Pose3d.struct);

    double actualTof = trajectory.length / OPTIMIZER_RESOLUTION;
    double error = (actualTof - tof) / actualTof;
    LoggingUtils.log("Shooting/Velocity Analyzer/Error", error);
    return error;
  }
}
