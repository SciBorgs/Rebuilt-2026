package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.SHOOTING_ANGLE_OFFSET;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public class ShotOptimizer {
  private static final double SPEED_CONSTANT = 0.01;
  private static final double ANGLE_INCREMENT = Math.PI / 12;

  private static final int TRAJECTORY_RESOLUTION = 100;
  private static final int OPTIMIZATION_RESOLUTION = 200;
  private static final int TRAJECTORY_SIZE_LIMIT = 200;

  private static final double[] GOAL = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

  private static final double MAX_SPEED = 20;
  private static final double MAXIMUM_ANGLE =
      SHOOTING_ANGLE_OFFSET.in(Radians) - MIN_ANGLE.in(Radians);
  private static final double MINIMUM_ANGLE =
      SHOOTING_ANGLE_OFFSET.in(Radians) - MAX_ANGLE.in(Radians);

  private static final int DISTANCE = 0, SPEED = 1, ANGLE = 2;

  private static Pose3d[] displayedTrajectory = new Pose3d[0];
  private static final Projectile projectile =
      new Fuel().config(TRAJECTORY_RESOLUTION, true, true, false, false);

  public static Command optimizeLaunch(double distance) {
    return Commands.runOnce(() -> logTrajectory(calculateLaunchParameters(distance)));
  }

  protected static double[] calculateLaunchParameters(double distance) {
    Tracer.startTrace("Shot Optimization");

    double speed = 0;
    double angle = 0;

    for (double testAngle = MINIMUM_ANGLE;
        testAngle < MAXIMUM_ANGLE;
        testAngle += ANGLE_INCREMENT) {
      double[][] trajectory = trajectory(distance, MAX_SPEED, testAngle);
      double[] finalTranslation = trajectory[trajectory.length - 1];

      // TEST IF SHOT IS POSSIBLE
      if (finalTranslation[X] - GOAL[X] < 0 || finalTranslation[Z] < GOAL[Z]) continue;
      speed = optimizeSpeed(distance, MAX_SPEED, testAngle);
      angle = testAngle;

      break;
    }

    Tracer.endTrace();
    return new double[] {distance, speed, angle};
  }

  private static double optimizeSpeed(double distance, double startingSpeed, double angle) {
    int iterations = 0;
    double speed = startingSpeed;

    while (iterations < OPTIMIZATION_RESOLUTION) {
      double[][] trajectory = trajectory(distance, speed, angle);

      double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDisplacement > 0) speed -= SPEED_CONSTANT * finalDistance;
      if (finalDisplacement < 0) speed += SPEED_CONSTANT * finalDistance;
      iterations++;
    }

    return speed;
  }

  private static double[][] trajectory(double distance, double speed, double angle) {
    projectile.reset();
    List<double[]> poseList = new ArrayList<>();

    projectile.launch(
        new double[] {GOAL[X] - distance, GOAL[Y], ROBOT_TO_SHOOTER.getZ()},
        new double[] {Math.cos(angle) * speed, 0, Math.sin(angle) * speed},
        new double[4],
        0);

    int frames = 0;
    while (!projectile.willMiss() && !projectile.willScore()) {
      frames++;
      if (frames >= TRAJECTORY_SIZE_LIMIT) break;

      poseList.add(projectile.translation.clone());
      projectile.periodic();
    }

    return poseList.toArray(new double[0][]);
  }

  private static void logTrajectory(double[] launchParameters) {
    double[][] trajectory =
        trajectory(launchParameters[DISTANCE], launchParameters[SPEED], launchParameters[ANGLE]);

    displayedTrajectory = new Pose3d[trajectory.length];
    for (int index = 0; index < trajectory.length; index++)
      displayedTrajectory[index] =
          new Pose3d(
              trajectory[index][X], trajectory[index][Y], trajectory[index][Z], new Rotation3d());
  }

  public static void updateLogging() {
    if (displayedTrajectory.length == 0) return;
    LoggingUtils.log("Shot Optimizer/Trajectory", displayedTrajectory, Pose3d.struct);
  }
}
