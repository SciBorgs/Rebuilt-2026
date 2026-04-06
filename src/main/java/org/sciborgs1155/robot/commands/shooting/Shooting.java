package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.lib.ProjectileVisualizer.X;
import static org.sciborgs1155.lib.ProjectileVisualizer.Y;
import static org.sciborgs1155.lib.ProjectileVisualizer.Z;
import static org.sciborgs1155.lib.ProjectileVisualizer.norm;
import static org.sciborgs1155.robot.Constants.EPS;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fieldRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fromLaunchParameters;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotToShooter;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterVelocity;
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
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.HOOD_ERROR_THRESHOLD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SCORE_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SCORE_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.SHOOTER_ERROR_THRESHOLD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.TURRET_ERROR_THRESHOLD;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.LAUNCH_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.MAX_AIR_TIME;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.SHOOTING_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.TRAJECTORY_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VisualizerConstants.VISUALIZER_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.toHoodAngle;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.toPitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
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

  private double speed;
  private double pitch;
  private double yaw;

  private double distance;
  private double shooterError;
  private double turretError;
  private double hoodError;

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
   * Drives the robot while shooting at the HUB. The turret is controlled with position setpoints as
   * opposed to velocity setpoints in dynamic control.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param omega A supplier for the angular velocity of the robot.
   */
  public Command runDiscreteShooter(InputStream vx, InputStream vy, InputStream omega) {
    return Commands.parallel(
            Commands.startEnd(
                () -> algorithm = Algorithm.DISCRETE, () -> algorithm = Algorithm.NONE),
            Commands.run(() -> update(vx.get(), vy.get(), omega.get())),
            hopper.intake().alongWith(indexer.forward()).onlyWhile(shouldIndex()),
            shooter.runShooter(() -> 0),
            turret.goToYaw(() -> Rotation2d.fromRadians(yaw)),
            hood.goToShootingAngle(() -> pitch),
            drive.drive(vx, vy, omega))
        .withName("Discrete Shooter");
  }

  /**
   * Drives the robot while shooting at the HUB. The turret is controlled with velocity setpoints as
   * opposed to position setpoints in discrete control.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param omega A supplier for the angular velocity of the robot.
   */
  public Command runDynamicShooter(InputStream vx, InputStream vy, InputStream omega) {
    return Commands.parallel(
            Commands.startEnd(
                () -> algorithm = Algorithm.DYNAMIC, () -> algorithm = Algorithm.NONE),
            Commands.run(() -> update(vx.get(), vy.get(), omega.get())),
            hopper.intake().alongWith(indexer.forward()).onlyWhile(shouldIndex()),
            shooter.runShooter(() -> 0),
            turret.goToYaw(() -> Rotation2d.fromRadians(yaw)),
            hood.goToShootingAngle(() -> pitch),
            drive.drive(vx, vy, omega))
        .withName("Dynamic Shooter");
  }

  /**
   * True if the robot is moving, false if the robot is not moving.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param omega A supplier for the angular velocity of the robot.
   */
  public Trigger discrete(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return new Trigger(
        () ->
            Math.abs(vx.getAsDouble()) < EPS
                && Math.abs(vy.getAsDouble()) < EPS
                && Math.abs(omega.getAsDouble()) < EPS);
  }

  /** Whether or not to run the indexer/hopper. */
  @SuppressWarnings("PMD.LinguisticNaming")
  public Trigger shouldIndex() {
    return new Trigger(
        () -> {
          // TURRET ERROR
          double turretAbsolutePosition = MathUtil.angleModulus(turret.position());
          double absoluteYaw = MathUtil.angleModulus(yaw);

          turretError = Math.abs(turretAbsolutePosition - absoluteYaw) * 100 / (Math.PI * 2);

          // HOOD ERROR
          double hoodAbsolutePosition = MathUtil.angleModulus(hood.angle());
          double absoluteHoodAngle = toHoodAngle(pitch);

          hoodError = Math.abs(hoodAbsolutePosition - absoluteHoodAngle) * 100 / (Math.PI * 2);

          return turretError < TURRET_ERROR_THRESHOLD
              && shooterError < SHOOTER_ERROR_THRESHOLD
              && hoodError < HOOD_ERROR_THRESHOLD
              && distance <= MAX_DISTANCE
              && distance >= MIN_DISTANCE;
        });
  }

  private void updateLaunchParameters(
      double robotX, double robotY, double heading, double vx, double vy, double omega) {
    double[] robotToShooter = robotToShooter(heading);

    double x = GOAL[X] - robotToShooter[X] - robotX;
    double y = GOAL[Y] - robotToShooter[Y] - robotY;

    double distance = Math.sqrt(x * x + y * y);
    double stationaryYaw = Math.atan2(y, x) - heading;

    double[] robotRelativeStationaryShotVelocity =
        robotRelativeShotVelocity(
            ParameterLookup.speed(distance), ParameterLookup.pitch(distance), stationaryYaw);

    double[] fieldRelativeStationaryShotVelocity =
        fieldRelative(robotRelativeStationaryShotVelocity, heading);
    double[] shooterVelocity = shooterVelocity(-vx, -vy, omega, heading);

    double[] fieldRelativeMovingShotVelocity = {
      fieldRelativeStationaryShotVelocity[X] - shooterVelocity[X],
      fieldRelativeStationaryShotVelocity[Y] - shooterVelocity[Y],
      fieldRelativeStationaryShotVelocity[Z] - shooterVelocity[Z]
    };
    double[] shotVelocity = robotRelative(fieldRelativeMovingShotVelocity, heading);

    speed = MathUtil.clamp(norm(shotVelocity), MIN_SPEED, MAX_SPEED);
    pitch = MathUtil.clamp(Math.asin(shotVelocity[Z] / speed), MIN_PITCH, MAX_PITCH);
    yaw = MathUtil.clamp(Math.atan2(shotVelocity[Y], shotVelocity[X]), MIN_YAW, MAX_YAW);
  }

  private void update(double vx, double vy, double omega) {
    double robotX = drive.pose().getX();
    double robotY = drive.pose().getY();
    double heading = drive.heading().getRadians();

    // DISTANCE CALCULATION
    double[] robotToShooter = robotToShooter(heading);

    double x = GOAL[X] - robotToShooter[X] - robotX;
    double y = GOAL[Y] - robotToShooter[Y] - robotY;

    distance = Math.sqrt(x * x + y * y);
    updateLaunchParameters(robotX, robotY, heading, vx, vy, omega);
  }

  /** Creates a visualizer that utilizes the subsystem positions to predict a trajectory. */
  public ProjectileVisualizer createVisualizer() {
    return fromLaunchParameters(
            () -> speed, () -> toPitch(hood.angle()), () -> turret.position(), drive, turret)
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
    return fromLaunchParameters(() -> speed, () -> pitch, () -> yaw, drive, turret)
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

    LoggingUtils.log("Shooting/Mechanism Error/Shooter", shooterError);
    LoggingUtils.log("Shooting/Mechanism Error/Turret", turretError);
    LoggingUtils.log("Shooting/Mechanism Error/Hood", hoodError);

    LoggingUtils.log(
        "Shooting/Mechanism Setpoints/Shooter", shooterError < SHOOTER_ERROR_THRESHOLD);
    LoggingUtils.log("Shooting/Mechanism Setpoints/Turret", turretError < TURRET_ERROR_THRESHOLD);
    LoggingUtils.log("Shooting/Mechanism Setpoints/Hood", hoodError < HOOD_ERROR_THRESHOLD);
  }
}
