package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;
import org.sciborgs1155.robot.hood.HoodConstants;

public class ShotOptimizer {
  protected static final double RESOLUTION = 100;
  protected static final int MAX_ITERATIONS = 200;
  protected static final double[] GOAL = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

  protected static final double MAX_SPEED = 20;
  protected static final double MAX_ANGLE =
      HoodConstants.MAX_ANGLE.in(Radians) + HoodConstants.SHOOTING_ANGLE_OFFSET.in(Radians);
  protected static final double MIN_ANGLE =
      HoodConstants.MIN_ANGLE.in(Radians) + HoodConstants.SHOOTING_ANGLE_OFFSET.in(Radians);

  protected static final double STARTING_DISTANCE = 5;
  protected static double finalDistance;
  protected static final double K_ANGLE = 0.01;
  protected static final double K_SPEED = 0.01;
  protected static final double TOLERANCE = 0.01;

  protected static double[][] trajectory = new double[0][];
  protected static double[] launchTranslation =
      new double[] {GOAL[X] - STARTING_DISTANCE, GOAL[Y], ROBOT_TO_SHOOTER.getZ()};

  protected static double launchSpeed = 10;
  protected static double launchAngle = Math.PI / 4;

  protected static double[] launchRotation = new double[4];
  protected static double launchRotationalVelocity;

  protected static final Projectile projectile =
      new Fuel().config(RESOLUTION, true, true, false, false);

  protected static final FuelVisualizer visualizer =
      new FuelVisualizer(
          () -> launchTranslation,
          () -> launchVelocity(),
          () -> launchRotation,
          () -> launchRotationalVelocity);

  /**
   * Returns a command to optimize the launch angle and speed for the current distance.
   *
   * @return a command to optimize the launch angle and speed for the current distance
   */
  public static Command optimizeCommand() {
    return Commands.runOnce(ShotOptimizer::optimizeForAngle);
  }

  protected static void optimizeForSpeed() {
    int iterations = 0;
    double[] finalTranslation;
    double finalDisplacement;

    while (iterations < MAX_ITERATIONS) {
      iterations++;
      generateTrajectory();

      finalTranslation = trajectory[trajectory.length - 1];
      finalDisplacement = finalTranslation[X] - GOAL[X];
      finalDistance = Math.abs(finalDisplacement);

      if (finalDistance < TOLERANCE) return;
      if (finalDisplacement > 0) launchSpeed -= K_SPEED * finalDistance;
      if (finalDisplacement < 0) launchSpeed += K_SPEED * finalDistance;
      if (launchSpeed > MAX_SPEED) return;
    }
  }

  protected static void optimizeForAngle() {
    int iterations = 0;
    double[] finalTranslation;
    double finalDisplacement;

    while (iterations < MAX_ITERATIONS) {
      iterations++;
      generateTrajectory();

      finalTranslation = trajectory[trajectory.length - 1];
      finalDisplacement = finalTranslation[X] - GOAL[X];
      finalDistance = Math.abs(finalDisplacement);

      if (finalDistance < TOLERANCE) return;
      if (finalDisplacement > 0) launchAngle += K_ANGLE * finalDistance;
      if (finalDisplacement < 0) launchAngle -= K_ANGLE * finalDistance;
      if (launchAngle > MAX_ANGLE || launchAngle < MIN_ANGLE) return;
    }
  }

  protected static void generateTrajectory() {
    projectile.reset();
    List<double[]> poseList = new ArrayList<>();
    projectile.launch(
        launchTranslation, launchVelocity(), launchRotation, launchRotationalVelocity);

    while (!projectile.willMiss() && !projectile.willScore()) {
      poseList.add(projectile.translation.clone());
      projectile.periodic();
    }

    trajectory = poseList.toArray(new double[0][]);
  }

  /** Updates the displayed trajectory. */
  public static void updateSimulation() {
    visualizer.updateTrajectorySimulation();
  }

  /** Logs optimization data to NetworkTables. */
  public static void updateLogging() {
    visualizer.updateLogging();
    LoggingUtils.log("Shot Optimizer/Final Distance", finalDistance);
    LoggingUtils.log("Shot Optimizer/Launch Speed", launchSpeed);
    LoggingUtils.log("Shot Optimizer/Launch Angle", launchAngle);
  }

  public static double[] launchVelocity() {
    return FuelVisualizer.shotVelocity(launchSpeed, launchAngle, 0, new Pose3d());
  }
}
