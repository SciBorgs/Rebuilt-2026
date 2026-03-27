package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
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
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DRAG_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.GOAL;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LAUNCH_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LIFT_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_AIR_TIME;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SCORE_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SCORE_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SHOOTING_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TRAJECTORY_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.VISUALIZER_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.YAW;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.turret.Turret;
import org.sciborgs1155.robot.turret.TurretConstants;

/** A command factory for the shooting algorithm. */
public class Shooting {
  private final Turret turret;
  private final Hood hood;
  private final Drive drive;

  private String mode = "NONE";
  private final double[] discreteLaunchParameters = {
    0, HoodConstants.DEFAULT_ANGLE.in(Radians), TurretConstants.START_ANGLE.in(Radians)
  };

  /** A command factory for the shooting algorithm. */
  public Shooting(Turret turret, Hood hood, Drive drive) {
    this.turret = turret;
    this.hood = hood;
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
  public Command runDiscreteShooter(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return Commands.parallel(
            Commands.startEnd(() -> mode = "DISCRETE", () -> mode = "NONE"),
            Commands.run(() -> update(vx.getAsDouble(), vy.getAsDouble(), omega.getAsDouble())),
            drive.drive(vx, vy, omega),
            turret.goToYaw(() -> Rotation2d.fromRadians(discreteLaunchParameters[YAW])),
            hood.goToShootingAngle(
                () -> MathUtil.inputModulus(discreteLaunchParameters[PITCH], MIN_PITCH, MAX_PITCH)))
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
  public Command runDynamicShooter(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return Commands.parallel(
            Commands.startEnd(() -> mode = "DYNAMIC", () -> mode = "NONE"),
            Commands.run(() -> update(vx.getAsDouble(), vy.getAsDouble(), omega.getAsDouble())),
            drive.drive(vx, vy, omega),
            turret.goToYaw(() -> Rotation2d.fromRadians(discreteLaunchParameters[YAW])),
            hood.goToShootingAngle(
                () -> MathUtil.inputModulus(discreteLaunchParameters[PITCH], MIN_PITCH, MAX_PITCH)))
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

  private static double[] discreteLaunchParameters(
      double robotX, double robotY, double heading, double vx, double vy, double omega) {
    double[] robotToShooter = robotToShooter(heading);

    double x = GOAL[X] - robotToShooter[X] - robotX;
    double y = GOAL[Y] - robotToShooter[Y] - robotY;

    double distance = Math.sqrt(x * x + y * y);
    double yaw = Math.atan2(y, x) - heading;

    double[] robotRelativeShotVelocity =
        robotRelativeShotVelocity(
            new double[] {ParameterLookup.speed(distance), ParameterLookup.pitch(distance), yaw});

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

    double distance = Math.sqrt(x * x + y * y);

    // PARAMETER UPDATING
    double[] newLaunchParameters = discreteLaunchParameters(robotX, robotY, heading, vx, vy, omega);

    discreteLaunchParameters[SPEED] = newLaunchParameters[SPEED];
    discreteLaunchParameters[PITCH] = newLaunchParameters[PITCH];
    discreteLaunchParameters[YAW] = newLaunchParameters[YAW];

    // LOGGING
    LoggingUtils.log("Shooting/Mode", mode);
    LoggingUtils.log("Shooting/Possible", distance <= MAX_DISTANCE);
    LoggingUtils.log("Shooting/Discrete/SPEED", discreteLaunchParameters[SPEED]);
    LoggingUtils.log("Shooting/Discrete/PITCH", discreteLaunchParameters[PITCH]);
    LoggingUtils.log("Shooting/Discrete/YAW", discreteLaunchParameters[YAW]);
  }

  /** Creates a visualizer that utilizes the subsystem positions to predict a trajectory. */
  public ProjectileVisualizer createVisualizer() {
    return fromLaunchParameters(
            () -> discreteLaunchParameters[SPEED],
            () -> Math.PI / 2 - hood.angle(),
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
    return fromLaunchParameters(
            () -> discreteLaunchParameters[SPEED],
            () -> discreteLaunchParameters[PITCH],
            () -> discreteLaunchParameters[YAW],
            drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(
            1 / SHOOTING_SPEED, MAX_AIR_TIME, VISUALIZER_RESOLUTION, VISUALIZER_RESOLUTION)
        .config(LAUNCH_ENABLED, TRAJECTORY_ENABLED);
  }
}
