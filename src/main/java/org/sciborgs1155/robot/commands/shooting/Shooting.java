package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fieldRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fromLaunchParameters;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.launchParameters;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotToShooter;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterVelocity;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.EPS;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameters.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameters.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LaunchParameters.YAW;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.DRAG_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.LIFT_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_PITCH;
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
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.ShooterConstants;
import org.sciborgs1155.robot.turret.Turret;

/** A command factory for the shooting algorithm. */
@SuppressWarnings("PMD.LinguisticNaming")
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
  private final double[] discreteLaunchParameters;

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

    discreteLaunchParameters = new double[] {0, toHoodAngle(MIN_PITCH), 0};
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
            shooter.runShooter(() -> RollerTable.rollerSpeed(discreteLaunchParameters[SPEED])),
            turret.goToYaw(() -> Rotation2d.fromRadians(discreteLaunchParameters[YAW])),
            hood.goToShootingAngle(
                () -> MathUtil.inputModulus(discreteLaunchParameters[PITCH], MIN_PITCH, MAX_PITCH)),
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
            shooter.runShooter(() -> RollerTable.rollerSpeed(discreteLaunchParameters[SPEED])),
            turret.goToYaw(() -> Rotation2d.fromRadians(discreteLaunchParameters[YAW])),
            hood.goToShootingAngle(
                () -> MathUtil.inputModulus(discreteLaunchParameters[PITCH], MIN_PITCH, MAX_PITCH)),
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
  public Trigger shouldIndex() {
    return new Trigger(
        () -> {
          // SHOOTER ERROR
          double rads = RollerTable.rollerSpeed(discreteLaunchParameters[SPEED]);
          shooterError =
              Math.abs(shooter.velocity() - rads)
                  * 100
                  / ShooterConstants.MAX_VELOCITY.in(RadiansPerSecond);

          // TURRET ERROR
          double turretAbsolutePosition = MathUtil.angleModulus(turret.position());
          double absoluteYaw = MathUtil.angleModulus(discreteLaunchParameters[YAW]);

          turretError = Math.abs(turretAbsolutePosition - absoluteYaw) * 100 / (Math.PI * 2);

          // HOOD ERROR
          double hoodAbsolutePosition = MathUtil.angleModulus(hood.angle());
          double absoluteHoodAngle =
              toHoodAngle(MathUtil.angleModulus(discreteLaunchParameters[PITCH]));

          hoodError = Math.abs(hoodAbsolutePosition - absoluteHoodAngle) * 100 / (Math.PI * 2);

          return turretError < TURRET_ERROR_THRESHOLD
              && shooterError < SHOOTER_ERROR_THRESHOLD
              && hoodError < HOOD_ERROR_THRESHOLD
              && distance <= MAX_DISTANCE
              && distance >= MIN_DISTANCE;
        });
  }

  private static double[] discreteLaunchParameters(
      double robotX, double robotY, double heading, double vx, double vy, double omega) {
    double[] robotToShooter = robotToShooter(heading);

    double x = GOAL[X] - robotToShooter[X] - robotX;
    double y = GOAL[Y] - robotToShooter[Y] - robotY;

    double distance = Math.sqrt(x * x + y * y);
    double yaw = Math.atan2(y, x) - heading;

    double[] robotRelativeShotVelocity =
        robotRelativeShotVelocity(
            new double[] {ParameterTable.speed(distance), ParameterTable.pitch(distance), yaw});

    double[] stationaryShotVelocity = fieldRelative(robotRelativeShotVelocity, heading);
    double[] shooterVelocity = shooterVelocity(-vx, -vy, omega, heading);

    return launchParameters(
        robotRelative(
            new double[] {
              stationaryShotVelocity[X] - shooterVelocity[X],
              stationaryShotVelocity[Y] - shooterVelocity[Y],
              stationaryShotVelocity[Z] - shooterVelocity[Z]
            },
            heading));
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

    // PARAMETER UPDATING
    double[] newLaunchParameters = discreteLaunchParameters(robotX, robotY, heading, vx, vy, omega);

    discreteLaunchParameters[SPEED] = newLaunchParameters[SPEED];
    discreteLaunchParameters[PITCH] = newLaunchParameters[PITCH];
    discreteLaunchParameters[YAW] = newLaunchParameters[YAW];
  }

  /** Creates a visualizer that utilizes the subsystem positions to predict a trajectory. */
  public ProjectileVisualizer createVisualizer() {
    return fromLaunchParameters(
            () -> RollerTable.speed(shooter.velocity()),
            () -> toPitch(hood.angle()),
            () -> turret.position(),
            drive,
            turret)
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
    return fromLaunchParameters(
            () -> discreteLaunchParameters[SPEED],
            () -> discreteLaunchParameters[PITCH],
            () -> discreteLaunchParameters[YAW],
            drive,
            turret)
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
    LoggingUtils.log("Shooting/Discrete/SPEED", discreteLaunchParameters[SPEED]);
    LoggingUtils.log("Shooting/Discrete/PITCH", discreteLaunchParameters[PITCH]);
    LoggingUtils.log("Shooting/Discrete/YAW", discreteLaunchParameters[YAW]);

    LoggingUtils.log("Shooting/Mechanism Error/Shooter", shooterError);
    LoggingUtils.log("Shooting/Mechanism Error/Turret", turretError);
    LoggingUtils.log("Shooting/Mechanism Error/Hood", hoodError);

    LoggingUtils.log(
        "Shooting/Mechanism Setpoints/Shooter", shooterError < SHOOTER_ERROR_THRESHOLD);
    LoggingUtils.log("Shooting/Mechanism Setpoints/Turret", turretError < TURRET_ERROR_THRESHOLD);
    LoggingUtils.log("Shooting/Mechanism Setpoints/Hood", hoodError < HOOD_ERROR_THRESHOLD);
  }
}
