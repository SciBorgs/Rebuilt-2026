package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.SHOOTING_ANGLE_OFFSET;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;

public class ShotOptimizer {
  protected static final double RESOLUTION = 100;
  protected static final int MAX_ITERATIONS = 200;
  protected static final double[] GOAL = Projectile.fromTranslation(Hub.TOP_CENTER_POINT);

  protected static final double MAX_SPEED = 20;
  protected static final double MAXIMUM_ANGLE = MAX_ANGLE.in(Radians) + SHOOTING_ANGLE_OFFSET.in(Radians);
  protected static final double MINIMUM_ANGLE = MIN_ANGLE.in(Radians) + SHOOTING_ANGLE_OFFSET.in(Radians);
  
  protected static final double K_ANGLE = 0.01;
  protected static final double K_SPEED = 0.01;
  protected static final double TOLERANCE = 0.01;

  protected static final Projectile projectile =
      new Fuel().config(RESOLUTION, true, true, false, false);

  protected static double optimizeForSpeed(double distance, double startingSpeed, double angle) {
    int iterations = 0;
    double speed = startingSpeed;

    while (iterations < MAX_ITERATIONS) {
      iterations++;
      if (speed > MAX_SPEED) return MAX_SPEED;
      double[][] trajectory = trajectory(distance, startingSpeed, angle);

      double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDistance < TOLERANCE) return speed;
      if (finalDisplacement > 0) speed -= K_SPEED * finalDistance;
      if (finalDisplacement < 0) speed += K_SPEED * finalDistance;
    }

    return speed;
  }

  protected static double optimizeForAngle(double distance, double startingAngle, double speed) {
    int iterations = 0;
    double angle = startingAngle;

    while (iterations < MAX_ITERATIONS) {
      iterations++;
      if (angle > MAXIMUM_ANGLE) return MAXIMUM_ANGLE;
      if (angle < MINIMUM_ANGLE) return MINIMUM_ANGLE;
      double[][] trajectory = trajectory(distance, speed, startingAngle);

      double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDistance < TOLERANCE) return angle;
      if (finalDisplacement > 0) angle += K_ANGLE * finalDistance;
      if (finalDisplacement < 0) angle -= K_ANGLE * finalDistance;
    }

    return angle;
  }

  protected static double[][] trajectory(double distance, double speed, double angle) {
    projectile.reset();
    List<double[]> poseList = new ArrayList<>();
    projectile.launch(new double[] {GOAL[X] - distance, GOAL[Y], ROBOT_TO_SHOOTER.getZ()}, launchVelocity(speed, angle), new double[4], 0);

    while (!projectile.willMiss() && !projectile.willScore()) {
      poseList.add(projectile.translation.clone());
      projectile.periodic();
    }

    return poseList.toArray(new double[0][]);
  }

  public static void updateLogging() {
  }

  public static double[] launchVelocity(double speed, double angle) {
    return FuelVisualizer.shotVelocity(speed, angle, 0, new Pose3d());
  }
}
