package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.lib.ProjectileVisualizer.X;
import static org.sciborgs1155.lib.ProjectileVisualizer.Y;
import static org.sciborgs1155.lib.ProjectileVisualizer.Z;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.ProjectileVisualizer;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.turret.Turret;

/** A command factory for the shooting algorithm. */
public class Shooting {
  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Hopper hopper;
  private final Indexer indexer;
  private final Drive drive;

  private enum Algorithm {
    DISCRETE,
    DYNAMIC,
    NONE
  }

  private Algorithm algorithm = Algorithm.NONE;
  private double distance, speed, pitch, yaw, rollerSpeed;

  /** A command factory for the shooting algorithm. */
  public Shooting(
      Shooter shooter, Turret turret, Hood hood, Hopper hopper, Indexer indexer, Drive drive) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.hopper = hopper;
    this.indexer = indexer;
    this.drive = drive;
  }

  /**
   * Drives the robot while shooting at the HUB. The turret is controlled with position setpoints.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param omega A supplier for the angular velocity of the robot.
   */
  public Command runShooter(InputStream vx, InputStream vy, InputStream omega) {
    return Commands.parallel(
            Commands.startEnd(
                () -> algorithm = Algorithm.DISCRETE, () -> algorithm = Algorithm.NONE),
            Commands.run(() -> updateLaunchParameters(vx.get(), vy.get(), omega.get())),
            hopper.intake().alongWith(indexer.forward()).onlyWhile(shouldIndex()),
            shooter.runShooter(() -> rollerSpeed),
            turret.goToYaw(() -> Rotation2d.fromRadians(yaw)),
            hood.goToShootingAngle(() -> pitch),
            drive.drive(vx, vy, omega))
        .withName("Shooting");
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
   * Updates the launch parameter fields based on th current state of the robot.
   *
   * @param vx the velocity of the robot along the x axis (from controller)
   * @param vy the velocity of the robot along the y axis (from controller)
   * @param omega A supplier for the angular velocity of the robot (from controller)
   */
  private void updateLaunchParameters(double vx, double vy, double omega) {
    LoggingUtils.log("Shooting/Input/vx", vx);
    LoggingUtils.log("Shooting/Input/vy", vy);
    LoggingUtils.log("Shooting/Input/omega", omega);

    double robotX = drive.pose().getX();
    double robotY = drive.pose().getY();

    double robotVx = drive.fieldRelativeChassisSpeeds().vxMetersPerSecond;
    double robotVy = drive.fieldRelativeChassisSpeeds().vyMetersPerSecond;

    double heading = drive.heading().getRadians();
    double robotOmega = drive.omega();

    double[] currentLaunchParameters =
        completeLaunchParameters(robotX, robotY, heading, robotVx, robotVy, robotOmega);

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
      double robotOmega) {
    double[] robotToShooter = robotToShooter(heading);
    double[] shooterTranslation = {robotToShooter[X] + robotX, robotToShooter[Y] + robotY};
    double[] hubToShooter = {GOAL[X] - shooterTranslation[X], GOAL[Y] - shooterTranslation[Y]};

    double distance = Math.hypot(hubToShooter[X], hubToShooter[Y]);
    double directPitch = ParameterLookup.pitch(distance);
    double directSpeed = ParameterLookup.speed(distance);
    double stationaryYaw = Math.atan2(hubToShooter[Y], hubToShooter[X]) - heading;

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
    LoggingUtils.log("Shooting/Algorithm", algorithm);
    LoggingUtils.log("Shooting/Possible", distance <= MAX_DISTANCE && distance >= MIN_DISTANCE);
    LoggingUtils.log("Shooting/Discrete/SPEED", speed);
    LoggingUtils.log("Shooting/Discrete/PITCH", pitch);
    LoggingUtils.log("Shooting/Discrete/YAW", yaw);

    LoggingUtils.log("Shooting/Mechanism Setpoints/Shooter", shooter.atSetpoint());
    LoggingUtils.log("Shooting/Mechanism Setpoints/Turret", turret.atGoal());
    LoggingUtils.log("Shooting/Mechanism Setpoints/Hood", hood.atGoal());
  }
}
