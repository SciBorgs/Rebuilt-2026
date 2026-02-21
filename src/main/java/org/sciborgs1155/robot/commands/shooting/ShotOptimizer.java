package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;

import java.util.ArrayList;
import java.util.List;

import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;
import org.sciborgs1155.robot.hood.HoodConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShotOptimizer {
  protected static final double RESOLUTION = 100;
  protected static final Projectile projectile = new Fuel().config(RESOLUTION, true, true, false, false);

  protected static double startingDistance = 5;
  protected static double finalDistance = 0;
  protected static final double KX = 0.01;
  protected static final double KZ = 0.01;
  protected static final double TOLERANCE = 0.01;
  protected static final int MAX_ITERATIONS = 200;
  protected static double[][] trajectory = new double[0][];
  protected static final double[] GOAL = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

  protected static double[] launchTranslation = new double[] {GOAL[X] - startingDistance, GOAL[Y], ROBOT_TO_SHOOTER.getZ()};
  protected static double[] launchVelocity = josephsAlgorithm(2, launchTranslation, GOAL);
  protected static final double[] launchRotation = new double[4];
  protected static final double launchRotationalVelocity = 0;
  protected static final FuelVisualizer visualizer = new FuelVisualizer(() -> launchTranslation, () -> launchVelocity, () -> launchRotation, () -> launchRotationalVelocity);

  protected static final double MAX_ANGLE = HoodConstants.MAX_ANGLE.in(Radians) + HoodConstants.SHOOTING_ANGLE_OFFSET.in(Radians);

  public static Command optimizeCommand() {
    return Commands.runOnce(ShotOptimizer::optimizeForZVelocity);
  }

  protected static void optimizeForXVelocity() {
    double iterations = 0;
    double[] finalTranslation = new double[3];
    double finalDisplacement = 0;

    while (iterations < MAX_ITERATIONS) {
      iterations++;
      generateTrajectory();

      finalTranslation = trajectory[trajectory.length - 1];
      finalDisplacement = finalTranslation[X] - GOAL[X];
      finalDistance = Math.abs(finalDisplacement);
    
      if (finalDistance < TOLERANCE) return;
      if (finalDisplacement > 0) launchVelocity[X] -= KX * finalDistance;
      if (finalDisplacement < 0) launchVelocity[X] += KX * finalDistance;
    }
  }

  protected static void optimizeForZVelocity() {
    double iterations = 0;
    double[] finalTranslation = new double[3];
    double finalDisplacement = 0;

    while (iterations < MAX_ITERATIONS) {
      iterations++;
      generateTrajectory();

      finalTranslation = trajectory[trajectory.length - 1];
      finalDisplacement = finalTranslation[X] - GOAL[X];
      finalDistance = Math.abs(finalDisplacement);
    
      if (finalDistance < TOLERANCE) return;
      if (finalDisplacement > 0) launchVelocity[Z] -= KZ * finalDistance;
      if (finalDisplacement < 0) launchVelocity[Z] += KZ * finalDistance;
    }
  }

  protected static void generateTrajectory() {
    projectile.reset();
    List<double[]> poseList = new ArrayList<>();
    projectile.launch(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);

    while (!projectile.willMiss() && !projectile.willScore()) {
      poseList.add(projectile.translation.clone());
      projectile.periodic();
    }

    trajectory = poseList.toArray(new double[0][]);
  }

  public static void updateSimulation() {
    visualizer.updateTrajectorySimulation();
  }

  public static void updateLogging() {
    visualizer.updateLogging();
    LoggingUtils.log("Shot Optimizer/Final Distance", finalDistance);
  }

  public static double[] josephsAlgorithm(double arcConstant, double[] starting, double[] goal) {
    double zStart = starting[Z] + ROBOT_TO_SHOOTER.getZ();
    double zTarget = goal[Z];

    double dx = goal[X] - starting[X];
    double dy = goal[Y] - starting[Y];

    double horizontalDist = Math.hypot(dx, dy);

    double zApex = Math.max(zStart, zTarget) + arcConstant;
    double vZ = Math.sqrt(2 * -Fuel.GRAVITY * (zApex - zStart));
    
    double timeToPeak = vZ / -Fuel.GRAVITY;
    double timeToFall = Math.sqrt(2 * (zApex - zTarget) / -Fuel.GRAVITY);
    double totalTime = timeToPeak + timeToFall;

    double vXY = horizontalDist / totalTime;

    double unitX = dx / horizontalDist;
    double unitY = dy / horizontalDist;

    return new double[] {unitX * vXY, unitY * vXY, vZ};
  }
}
