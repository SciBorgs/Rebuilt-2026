package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.ProjectileVisualizer.X;
import static org.sciborgs1155.lib.ProjectileVisualizer.Y;
import static org.sciborgs1155.lib.ProjectileVisualizer.Z;
import static org.sciborgs1155.lib.ProjectileVisualizer.fromTranslation;
import static org.sciborgs1155.lib.ProjectileVisualizer.norm;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fieldRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fromLaunchParameters;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotToShooter;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterVelocity;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.YAW;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.velocityLookup;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.DRAG_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.LIFT_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_YAW;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_YAW;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.GOAL;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SCORE_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SCORE_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.LAUNCH_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.MAX_AIR_TIME;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.SHOOTING_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.TRAJECTORY_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.VISUALIZER_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.toPitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Function;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.ProjectileVisualizer;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.slapdown.Slapdown;
import org.sciborgs1155.robot.turret.Turret;

/** A command factory for the shooting algorithm. */
public class Shooting {
  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Hopper hopper;
  private final Indexer indexer;
  private final Slapdown slapdown;
  private final Drive drive;

  private boolean running;
  private double distance, speed, pitch, yaw, rollerSpeed;

  /** A command factory for the shooting algorithm. */
  public Shooting(
      Shooter shooter,
      Turret turret,
      Hood hood,
      Hopper hopper,
      Indexer indexer,
      Slapdown slapdown,
      Drive drive) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.hopper = hopper;
    this.indexer = indexer;
    this.slapdown = slapdown;
    this.drive = drive;
  }

  /** Shoots at the HUB when at setpoint. */
  public Command scoreHub() {
    return Commands.parallel(
            Commands.startEnd(() -> running = true, () -> running = false),
            Commands.run(() -> updateLaunchParameters(Hub.TOP_CENTER_POINT)),
            hopper
                .intake()
                .alongWith(indexer.forward())
                .alongWith(slapdown.squeeze())
                .onlyWhile(shouldIndex()),
            shooter.runShooter(() -> rollerSpeed),
            turret.goToYaw(() -> Rotation2d.fromRadians(yaw)),
            hood.goToShootingAngle(() -> pitch))
        .withName("score hub");
  }

  /** Feeds to the left. */
  public Command feedLeft() {
    return Commands.parallel(
            Commands.startEnd(() -> running = true, () -> running = false),
            Commands.run(() -> updateLaunchParameters(Hub.LEFT_FEED)),
            hopper
                .intake()
                .alongWith(indexer.forward())
                .alongWith(slapdown.squeeze())
                .onlyWhile(shouldIndex()),
            shooter.runShooter(() -> rollerSpeed),
            turret.goToYaw(() -> Rotation2d.fromRadians(yaw)),
            hood.goToShootingAngle(() -> pitch))
        .withName("feed left");
  }

  /** Feeds to the right. */
  public Command feedRight() {
    return Commands.parallel(
            Commands.startEnd(() -> running = true, () -> running = false),
            Commands.run(() -> updateLaunchParameters(Hub.LEFT_FEED)),
            hopper
                .intake()
                .alongWith(indexer.forward())
                .alongWith(slapdown.squeeze())
                .onlyWhile(shouldIndex()),
            shooter.runShooter(() -> rollerSpeed),
            turret.goToYaw(() -> Rotation2d.fromRadians(yaw)),
            hood.goToShootingAngle(() -> pitch))
        .withName("feed right");
  }

  /** Whether or not to run the indexer/hopper. */
  public Trigger shouldIndex() {
    return new Trigger(
        () ->
            turret.atGoal()
                && shooter.atSetpoint()
                && hood.atGoal()
                && distance <= MAX_DISTANCE
                && distance >= MIN_DISTANCE);
  }

  /**
   * @return A Command to move the hood all the way down and idle the shooter.
   */
  public Command hideAway() {
    return Commands.parallel(shooter.idleShooter(), hood.goTo(HoodConstants.MIN_ANGLE))
        .withName("hide away");
  }

  /** Updates the launch parameter fields based on th current state of the robot. */
  private void updateLaunchParameters(Translation3d goal) {
    double robotX = drive.pose().getX();
    double robotY = drive.pose().getY();

    double robotVx = drive.fieldRelativeChassisSpeeds().vxMetersPerSecond;
    double robotVy = drive.fieldRelativeChassisSpeeds().vyMetersPerSecond;

    double heading = drive.heading().getRadians();
    double robotOmega = drive.omega();

    double[] currentLaunchParameters =
        completeLaunchParameters(
            robotX, robotY, heading, robotVx, robotVy, robotOmega, fromTranslation(goal));

    distance = currentLaunchParameters[DISTANCE];
    speed = currentLaunchParameters[SPEED];
    pitch = currentLaunchParameters[PITCH];
    yaw = currentLaunchParameters[YAW];

    rollerSpeed = velocityLookup.rollerSpeed(speed);
  }

  /**
   * Computes a launch vector that will accurately shoot FUEL into the HUB.
   *
   * @param robotX the x-position of the robot, in meters
   * @param robotY the y-position of the robot, in meters
   * @param heading the heading of the robot, in radians
   * @param robotVx the x-velocity of the robot, in meters per second
   * @param robotVy the y-velocity of the robot, in meters per second
   * @param robotOmega the rotational velocity of the robot, in radians per second
   * @return a double[] containing {distance (meters), speed (meters per second), pitch (radians),
   *     and yaw (radians)}
   */
  private static double[] completeLaunchParameters(
      double robotX,
      double robotY,
      double heading,
      double robotVx,
      double robotVy,
      double robotOmega,
      double[] goal) {
    double[] robotToShooter = robotToShooter(heading);
    double[] shooterTranslation = {robotToShooter[X] + robotX, robotToShooter[Y] + robotY};
    double[] goalToShooter = {goal[X] - shooterTranslation[X], goal[Y] - shooterTranslation[Y]};

    double distance = Math.hypot(goalToShooter[X], goalToShooter[Y]);
    double directPitch = ParameterLookup.pitch(distance);
    double directSpeed = ParameterLookup.speed(distance);
    double stationaryYaw = Math.atan2(goalToShooter[Y], goalToShooter[X]) - heading;

    double[] shotVelocity = robotRelativeShotVelocity(directSpeed, directPitch, stationaryYaw);
    double[] fieldRelativeShotVelocity = fieldRelative(shotVelocity, heading);
    double[] shooterVelocity = shooterVelocity(robotVx, robotVy, robotOmega, heading);

    double[] trueShotVelocity =
        robotRelative(
            new double[] {
              fieldRelativeShotVelocity[X] - shooterVelocity[X],
              fieldRelativeShotVelocity[Y] - shooterVelocity[Y],
              fieldRelativeShotVelocity[Z] - shooterVelocity[Z]
            },
            heading);

    double speed = MathUtil.clamp(norm(trueShotVelocity), MIN_SPEED, MAX_SPEED);
    double pitch =
        MathUtil.clamp(
            Math.asin(trueShotVelocity[Z] / norm(trueShotVelocity)), MIN_PITCH, MAX_PITCH);
    double yaw =
        MathUtil.clamp(Math.atan2(trueShotVelocity[Y], trueShotVelocity[X]), MIN_YAW, MAX_YAW);

    return new double[] {distance, speed, pitch, yaw};
  }

  /**
   * @return A Trigger that activates when imminently crossing between field zones within
   *     HOOD_DOWN_TIME.
   */
  public Trigger crossingAlliance() {
    return new Trigger(
        () -> {
          Translation2d velocity = drive.velocity();
          Pose2d pose = drive.pose();

          double projectedDeltaX = velocity.getX() * HoodConstants.HOOD_DOWN_TIME.in(Seconds);
          double projectedDeltaY = velocity.getY() * HoodConstants.HOOD_DOWN_TIME.in(Seconds);

          double nearHubDisplacement = FieldConstants.LinesVertical.HUB_CENTER - pose.getX();
          double farHubDisplacement = FieldConstants.LinesVertical.OPP_HUB_CENTER - pose.getX();
          Function<Double, Boolean> compare =
              (Double displacement) ->
                  Math.abs(projectedDeltaX) > Math.abs(displacement)
                      && ((projectedDeltaX > 0) == (displacement > 0));
          boolean goingAroundHub =
              Math.abs(pose.getY() + projectedDeltaY - FieldConstants.LinesHorizontal.CENTER)
                  > (DriveConstants.RADIUS.in(Meters) + FieldConstants.Hub.WIDTH / 2);
          return (compare.apply(nearHubDisplacement) || compare.apply(farHubDisplacement))
              && goingAroundHub;
        });
  }

  /** Creates a visualizer that utilizes the subsystem positions to predict a trajectory. */
  public ProjectileVisualizer createVisualizer() {
    return fromLaunchParameters(
            () -> velocityLookup.launchSpeed(shooter.velocity()),
            () -> toPitch(hood.angle()),
            () -> turret.position(),
            drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(
            1 / SHOOTING_SPEED, MAX_AIR_TIME, VISUALIZER_RESOLUTION, VISUALIZER_RESOLUTION)
        .config(LAUNCH_ENABLED, TRAJECTORY_ENABLED);
  }

  /**
   * Creates a FuelVisualizer with the settings used to generate shots for the shooting algorithm.
   */
  public ProjectileVisualizer createVectorVisualizer() {
    return fromLaunchParameters(() -> speed, () -> pitch, () -> yaw, drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(
            1 / SHOOTING_SPEED, MAX_AIR_TIME, VISUALIZER_RESOLUTION, VISUALIZER_RESOLUTION)
        .config(LAUNCH_ENABLED, TRAJECTORY_ENABLED);
  }

  /** Logs shooting data to NetworkTables. */
  public void updateLogging() {
    LoggingUtils.log("Shooting/Running", running);
    LoggingUtils.log("Shooting/Distance", distance);
    LoggingUtils.log("Shooting/Possible", distance <= MAX_DISTANCE && distance >= MIN_DISTANCE);
    LoggingUtils.log("Shooting/Discrete/SPEED", speed);
    LoggingUtils.log("Shooting/Discrete/PITCH", pitch);
    LoggingUtils.log("Shooting/Discrete/YAW", yaw);

    LoggingUtils.log("Shooting/Mechanism Setpoints/Shooter", shooter.atSetpoint());
    LoggingUtils.log("Shooting/Mechanism Setpoints/Turret", turret.atGoal());
    LoggingUtils.log("Shooting/Mechanism Setpoints/Hood", hood.atGoal());
  }
}
