package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.PITCH;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.SPEED;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.YAW;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants;
import org.sciborgs1155.robot.turret.Turret;
import org.sciborgs1155.robot.turret.TurretConstants;

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

  private Command updateLaunchParameters() {
    return Commands.run(
        () -> {
          launchParameters =
              ShotGenerator.movingLaunchParameters(
                  drive.pose3d(), drive.fieldRelativeChassisSpeeds());
          LoggingUtils.log("Shooting/Parameters/SPEED", launchParameters[SPEED]);
          LoggingUtils.log("Shooting/Parameters/PITCH", launchParameters[PITCH]);
          LoggingUtils.log("Shooting/Parameters/YAW", launchParameters[YAW]);
        });
  }

  /**
   * Simultaneously calculates new launch parameters and passes those parameters into the
   * subsystems.
   */
  public Command runShooter() {
    return Commands.parallel(
        updateLaunchParameters(),
        turret.goTo(() -> launchParameters[YAW]),
        hood.goTo(() -> Math.PI / 2 - launchParameters[PITCH]));
  }

  /** Creates a visualizer that utilizes the subsystem positions to predict a trajectory. */
  public ProjectileVisualizer createVisualizer() {
    return FuelVisualizer.fromLaunchParameters(
        () -> launchParameters[SPEED],
        () -> Math.PI / 2 - hood.angle(),
        () -> turret.position(),
        drive);
  }
}
