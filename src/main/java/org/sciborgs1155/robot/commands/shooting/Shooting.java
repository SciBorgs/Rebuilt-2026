package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fieldRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.fromLaunchParameters;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.launchParameters;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelative;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotRelativeShotVelocity;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.robotToShooter;
import static org.sciborgs1155.robot.commands.shooting.FuelVisualizer.shooterVelocity;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.DRAG_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.GOAL;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LAUNCH_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LAUNCH_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.LIFT_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.MAX_AIR_TIME;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SCORE_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SCORE_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SHOOTING_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TRAJECTORY_ENABLED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.TRAJECTORY_RESOLUTION;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.YAW;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants;
import org.sciborgs1155.robot.turret.Turret;
import org.sciborgs1155.robot.turret.TurretConstants;

/** A command factory for the shooting algorithm. */
public class Shooting {
  private final Turret turret;
  private final Hood hood;
  private final Drive drive;

  private double[] launchParameters = {
    ShooterConstants.IDLE_VELOCITY.in(RadiansPerSecond),
    HoodConstants.DEFAULT_ANGLE.in(Radians),
    TurretConstants.START_ANGLE.in(Radians)
  };

  /** A command factory for the shooting algorithm. */
  public Shooting(Turret turret, Hood hood, Drive drive) {
    this.turret = turret;
    this.hood = hood;
    this.drive = drive;
  }

  /** Calculates the launch parameters required to shoot on the move (speed, pitch, yaw). */
  public static double[] movingLaunchParameters(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double heading = robotPose.getRotation().getZ();
    double[] robotToShooter = robotToShooter(heading);

    double x = GOAL[X] - robotToShooter[X] - robotPose.getX();
    double y = GOAL[Y] - robotToShooter[Y] - robotPose.getY();

    double distance = Math.sqrt(x * x + y * y);
    double yaw = Math.atan2(y, x) - heading;

    double[] robotRelativeShotVelocity =
        robotRelativeShotVelocity(
            new double[] {ShotLookUpTable.speed(distance), ShotLookUpTable.pitch(distance), yaw});

    double[] stationaryShotVelocity = fieldRelative(robotRelativeShotVelocity, heading);
    double[] shooterVelocity =
        shooterVelocity(
            robotVelocity.vxMetersPerSecond,
            robotVelocity.vyMetersPerSecond,
            robotVelocity.omegaRadiansPerSecond,
            heading);

    return launchParameters(
        robotRelative(
            new double[] {
              stationaryShotVelocity[X] - shooterVelocity[X],
              stationaryShotVelocity[Y] - shooterVelocity[Y],
              stationaryShotVelocity[Z] - shooterVelocity[Z]
            },
            heading));
  }

  /**
   * Simultaneously calculates new launch parameters and passes those parameters into the
   * subsystems.
   */
  public Command runShooter() {
    return Commands.parallel(
        Commands.run(
            () -> {
              launchParameters =
                  movingLaunchParameters(drive.pose3d(), drive.fieldRelativeChassisSpeeds());
              LoggingUtils.log("Shooting/Parameters/SPEED", launchParameters[SPEED]);
              LoggingUtils.log("Shooting/Parameters/PITCH", launchParameters[PITCH]);
              LoggingUtils.log("Shooting/Parameters/YAW", launchParameters[YAW]);
            }),
        turret.goTo(() -> launchParameters[YAW]),
        hood.goTo(() -> Math.PI / 2 - launchParameters[PITCH]));
  }

  /** Creates a visualizer that utilizes the subsystem positions to predict a trajectory. */
  public ProjectileVisualizer createVisualizer() {
    return fromLaunchParameters(
            () -> launchParameters[SPEED],
            () -> Math.PI / 2 - hood.angle(),
            () -> turret.position(),
            drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(
            1 / SHOOTING_SPEED, MAX_AIR_TIME, TRAJECTORY_RESOLUTION, LAUNCH_RESOLUTION)
        .config(LAUNCH_ENABLED, TRAJECTORY_ENABLED);
  }

  /**
   * Creates a FuelVisualizer with the settings used to generate shots for the shooting algorithm.
   */
  public static ProjectileVisualizer createVectorVisualizer(Drive drive) {
    return fromLaunchParameters(
            () -> movingLaunchParameters(drive.pose3d(), drive.fieldRelativeChassisSpeeds()), drive)
        .withScoringParameters(GOAL, SCORE_RADIUS, SCORE_DEPTH)
        .configPhysics(true, DRAG_ENABLED, false, LIFT_ENABLED)
        .configGeneration(
            1 / SHOOTING_SPEED, MAX_AIR_TIME, TRAJECTORY_RESOLUTION, LAUNCH_RESOLUTION)
        .config(LAUNCH_ENABLED, TRAJECTORY_ENABLED);
  }
}
