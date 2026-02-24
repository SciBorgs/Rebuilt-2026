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
  private static final double RESOLUTION = 100;
  private static final int MAX_ITERATIONS = 200;
  private static final int MAX_FRAMES = 200;
  private static final double[] GOAL = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

  private static final double MAX_SPEED = 20;
  private static final double MAXIMUM_ANGLE =
      MAX_ANGLE.in(Radians) + SHOOTING_ANGLE_OFFSET.in(Radians);
  private static final double MINIMUM_ANGLE =
      MIN_ANGLE.in(Radians) + SHOOTING_ANGLE_OFFSET.in(Radians);

  private static final double K_P = 0.01;
  private static final double TOLERANCE = Math.PI / 12;
  private static final double ANGLE_INCREMENT = 0.1;

  private static Pose3d[] displayedTrajectory = new Pose3d[0];
  private static final Projectile projectile =
      new Fuel().config(RESOLUTION, true, true, false, false);

  public static Command optimizeLaunch(double distance) {
    return Commands.runOnce(() -> calculateLaunchParameters(distance));
  }

  protected static void calculateLaunchParameters(double distance) {
    Tracer.startTrace("Shot Optimization");
    if (!checkAngle(distance, Math.PI / 4)) throw new UnsupportedOperationException("Impossible shot!");
    
    for (double testAngle = Math.PI / 4; testAngle < Math.PI / 2 || testAngle <= MAXIMUM_ANGLE; testAngle += ANGLE_INCREMENT) 
    if (checkAngle(distance, testAngle)) {
      logTrajectory(trajectory(distance, optimizeSpeed(distance, MAX_SPEED, testAngle), testAngle));
      break;
    }
    
    Tracer.endTrace();
  }

  private static double optimizeSpeed(double distance, double startingSpeed, double angle) {
    if (!checkAngle(distance, angle)) return 0;

    int iterations = 0;
    double speed = startingSpeed;
    while (iterations < MAX_ITERATIONS) {
      iterations++;
      if (speed >= MAX_SPEED) return MAX_SPEED;
      double[][] trajectory = trajectory(distance, speed, angle);

      double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDistance < TOLERANCE) return speed;
      if (finalDisplacement > 0) speed -= K_P * finalDistance;
      if (finalDisplacement < 0) speed += K_P * finalDistance;
    }

    return speed;
  }

  private static boolean checkAngle(double distance, double angle) {
    double[][] trajectory = trajectory(distance, MAX_SPEED, angle);
    double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];

    return finalDisplacement > -TOLERANCE;
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
      if (frames >= MAX_FRAMES) break;
      poseList.add(projectile.translation.clone());
      projectile.periodic();
    }

    return poseList.toArray(new double[0][]);
  }

  private static void logTrajectory(double[][] trajectory) {
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
