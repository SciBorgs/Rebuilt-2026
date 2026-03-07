package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.*;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel.*;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.add3;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.norm3;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.sub3;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.toPose;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.SHOOTING_ANGLE_OFFSET;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer.Fuel;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile;
import org.sciborgs1155.robot.drive.Drive;

public final class ShotGenerator {
  private static final boolean WEIGHT_ENABLED = true;
  private static final boolean DRAG_ENABLED = true;
  private static final boolean TORQUE_ENABLED = false;
  private static final boolean LIFT_ENABLED = false;

  private static final double VELOCITY_DEADBAND = 0.1;
  private static final double DISTANCE_OFFSET = 0.05;
  // tuning?
  private static final double LATENCY = 0.05;

  private static final double MAX_AIR_TIME = 10;
  private static final int TRAJECTORY_RESOLUTION = 200;
  private static final int OPTIMIZATION_RESOLUTION = 500;
  private static final int TRAJECTORY_SIZE_LIMIT = 500;

  private static final double CLEARANCE = 0.13;
  private static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

  private static final double SCORE_DEPTH = 0;
  private static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

  private static final double SPEED_PRECISION = 0.005;
  private static final double PITCH_PRECISION = Math.PI / 96;

  private static final double MAX_SPEED = 20;
  static final double MAXIMUM_ANGLE = SHOOTING_ANGLE_OFFSET.in(Radians) - MIN_ANGLE.in(Radians);
  static final double MINIMUM_ANGLE = SHOOTING_ANGLE_OFFSET.in(Radians) - MAX_ANGLE.in(Radians);

  static final double[] GOAL = fromTranslation(Hub.TOP_CENTER_POINT);

  private static final Projectile PROJECTILE =
      new Fuel()
          .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
          .config(
              TRAJECTORY_RESOLUTION, WEIGHT_ENABLED, DRAG_ENABLED, TORQUE_ENABLED, LIFT_ENABLED);

  private ShotGenerator() {}

  private static double[][] generateDirectTrajectory(double[] launchParameters) {
    PROJECTILE.reset();
    List<double[]> poseList = new ArrayList<>();

    Pose3d robotPose =
        new Pose3d(
            GOAL[X] - launchParameters[DISTANCE] - ROBOT_TO_SHOOTER.getX(),
            GOAL[Y] - ROBOT_TO_SHOOTER.getY(),
            0,
            new Rotation3d());

    double[] shotVelocity = shotVelocity(launchParameters, robotPose.getRotation().getZ());
    double[] launchVelocity = launchVelocity(shotVelocity, robotPose, new ChassisSpeeds());
    double[] launchTranslation = launchTranslation(shotVelocity, robotPose);

    PROJECTILE.launch(launchTranslation, launchVelocity, new double[4], 0);

    int frames = 0;
    while (!PROJECTILE.willMiss() && !PROJECTILE.willScore()) {
      frames++;
      if (frames >= TRAJECTORY_SIZE_LIMIT) break;

      poseList.add(PROJECTILE.translation.clone());
      PROJECTILE.periodic();
    }

    return poseList.toArray(new double[0][]);
  }

  /** Optimizes the startingSpeed and returns the speed necessary to score. */
  private static double optimizeSpeed(double distance, double startingSpeed, double angle) {
    int iterations = 0;
    double speed = startingSpeed;

    while (iterations < OPTIMIZATION_RESOLUTION) {
      double[][] trajectory = generateDirectTrajectory(new double[] {distance, speed, angle, 0});

      double finalDisplacement = trajectory[trajectory.length - 1][X] - GOAL[X];
      double finalDistance = Math.abs(finalDisplacement);

      if (finalDisplacement > 0) speed -= SPEED_PRECISION * finalDistance;
      if (finalDisplacement < 0) speed += SPEED_PRECISION * finalDistance;
      iterations++;
    }

    return speed;
  }

  /** Checks if the current shot is possible. */
  private static boolean checkAngle(double distance, double angle) {
    double[][] trajectory = generateDirectTrajectory(new double[] {distance, MAX_SPEED, angle, 0});
    double[] finalTranslation = trajectory[trajectory.length - 1];

    return !(finalTranslation[X] - GOAL[X] < 0 || finalTranslation[Z] < GOAL[Z]);
  }

  /** Checks if the FUEL would clear the trajectory with the given launch parameters. */
  private static boolean checkClearance(double distance, double speed, double angle) {
    double[][] trajectory = generateDirectTrajectory(new double[] {distance, speed, angle, 0});
    for (int index = trajectory.length - 1; index >= 0; index--) {
      double[] translation = trajectory[index];

      if (translation[X] - GOAL[X] <= -CLEARANCE_CHECK) break;
      if (translation[Z] > GOAL[Z] + CLEARANCE) return true;
    }

    return false;
  }

  private static double[] stationaryLaunchParameters(double[] shooterPose, double heading) {
    double yDisplacement = GOAL[Y] - shooterPose[Y];
    double xDisplacement = GOAL[X] - shooterPose[X];

    double distance = Math.hypot(yDisplacement, xDisplacement) - DISTANCE_OFFSET;
    double yaw = Math.atan2(yDisplacement, xDisplacement) - heading;

    double[] launchParameters = TableGenerator.directLaunchParameters(distance);

    return new double[] {distance, launchParameters[SPEED], launchParameters[PITCH], yaw};
  }

  /**
   * Calculated the launch parameters required to shoot into the HUB from the given distance.
   *
   * @param distance the distance of the shooter from the HUB
   * @return the launch parameters required to shoot into the HUB
   */
  public static double[] optimizedLaunchParameters(double distance) {
    double speed = 0;
    double angle = 0;

    for (double testAngle = MINIMUM_ANGLE;
        testAngle < MAXIMUM_ANGLE;
        testAngle += PITCH_PRECISION) {

      // TEST IF SHOT IS POSSIBLE
      if (!checkAngle(distance, testAngle)) continue;

      double optimalSpeed = optimizeSpeed(distance, MAX_SPEED, testAngle);
      double optimalAngle = testAngle;

      // TEST IF FUEL HAS ENOUGH CLEARANCE OVER THE HUB EDGE
      if (checkClearance(distance, optimalSpeed, optimalAngle)) {
        speed = optimalSpeed;
        angle = optimalAngle;
        break;
      }
    }

    return new double[] {distance, speed, angle, 0};
  }

  /**
   * A utility method used to view the results of shot optimization.
   *
   * @param distance the distance from the HUB
   * @return a command to calculate and display the optimized trajectory
   */
  public static Command displayOptimizedShot(double distance, double robotVelocity) {
    double compensatedDistance = distance + (robotVelocity * LATENCY);

    return Commands.runOnce(
        () -> {
          double[] launchParameters = optimizedLaunchParameters(compensatedDistance);
          double[][] trajectory = generateDirectTrajectory(launchParameters);
          Pose3d[] poses = new Pose3d[trajectory.length];

          for (int index = 0; index < poses.length; index++)
            poses[index] = toPose(trajectory[index], 0);

          LoggingUtils.log("Shooting/Optimized Shot Display", poses, Pose3d.struct);
        });
  }

  /**
   * Calculates the launch parameters to score FUEL in the HUB.
   *
   * @param robotPose the pose of the robot
   * @param robotVelocity the velocity of the robot
   * @return the launch parameters [DISTANCE, SPEED, PITCH, YAW]
   */
  public static double[] movingLaunchParameters(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double heading = robotPose.getRotation().getZ();

    double[] shooterPose =
        add3(shooterPose(robotPose), fromTranslation(robotPose.getTranslation()));

    double[] compensatedShooter = {
      shooterPose[X] + (robotVelocity.vxMetersPerSecond * LATENCY),
      shooterPose[Y] + (robotVelocity.vyMetersPerSecond * LATENCY),
      shooterPose[Z]
    };
    Pose3d compensatedRobot =
        new Pose3d(
            new Translation3d(
                robotPose.getX() + (robotVelocity.vxMetersPerSecond * LATENCY),
                robotPose.getY() + (robotVelocity.vyMetersPerSecond * LATENCY),
                robotPose.getZ()),
            robotPose.getRotation());

    double[] stationaryLaunchParameters = stationaryLaunchParameters(compensatedShooter, heading);
    double[] stationaryShotVelocity = shotVelocity(stationaryLaunchParameters, heading);
    double[] shooterVelocity =
        shooterVelocity(stationaryShotVelocity, compensatedRobot, robotVelocity);
    double shooterSpeed = norm3(shooterVelocity);

    LoggingUtils.log("Shooting/Shooter Velocity", shooterSpeed);
    if (shooterSpeed < VELOCITY_DEADBAND) return stationaryLaunchParameters;

    double[] movingShotVelocity = sub3(stationaryShotVelocity, shooterVelocity);
    return launchParameters(movingShotVelocity, heading);
  }

  /**
   * Configures visualizer with settings specified by the Shot Generator.
   *
   * @param visualizer the visualizer to configure.
   * @return the configured visualizer.
   */
  public static ProjectileVisualizer configureVisualizer(FuelVisualizer visualizer) {
    return visualizer
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(WEIGHT_ENABLED, DRAG_ENABLED, TORQUE_ENABLED, LIFT_ENABLED)
        .configGeneration(0.05, MAX_AIR_TIME, TRAJECTORY_RESOLUTION, TRAJECTORY_RESOLUTION);
  }

  /**
   * Creates a new visualizer with input parameters calculated by the Shot Generator.
   *
   * @param drive the drivetrain subsystem
   * @return the configured visualizer.
   */
  public static ProjectileVisualizer createVisualizer(Drive drive) {
    return configureVisualizer(
        FuelVisualizer.fromLaunchParameters(
            () -> movingLaunchParameters(drive.pose3d(), drive.fieldRelativeChassisSpeeds()),
            drive));
  }
}
