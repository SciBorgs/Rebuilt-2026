package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.lib.shooting.ShootingAlgorithm;
import org.sciborgs1155.robot.drive.Drive;

/**
 * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
 *
 * @see Projectile
 */
public class FuelLaunchVisualizer extends LaunchVisualizer {
  /**
   * Creates a new FuelLaunchVisualizer with the given launch parameters. The visualizer will create
   * and launch projectiles with the given parameters when the launchProjectile command is executed,
   * and will update the simulation and logging for those projectiles when the updateSimulation and
   * updateLogging methods are called, respectively.
   *
   * @param launchTranslation a supplier that provides the current translation of the projectile at
   *     launch time
   * @param launchVelocity a supplier that provides the current velocity of the projectile at launch
   *     time
   * @param launchRotation a supplier that provides the current rotation of the projectile at launch
   *     time
   * @param launchRotationalVelocity a supplier that provides the current rotational velocity of the
   *     projectile at launch time
   */
  public FuelLaunchVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    super(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);
  }

  /**
   * Creates a new FuelLaunchVisualizer with the given launch parameters. The visualizer will create
   * and launch projectiles with the given parameters when the launchProjectile command is executed,
   * and will update the simulation and logging for those projectiles when the updateSimulation and
   * updateLogging methods are called, respectively.
   *
   * @param launchVelocity a supplier that provides the current velocity of the projectile at launch
   *     time
   * @param robotPose a supplier that provides the current pose of the robot at launch time
   * @param robotVelocity a supplier that provides the current velocity of the robot at launch time
   */
  public FuelLaunchVisualizer(
      Supplier<double[]> launchVelocity,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> Fuel.launchTranslation(launchVelocity.get(), robotPose.get()),
        () -> Fuel.launchVelocity(launchVelocity.get(), robotPose.get(), robotVelocity.get()),
        () -> Fuel.launchRotation(launchVelocity.get(), robotPose.get()),
        Fuel::launchRotationalVelocity);
  }

  /**
   * Creates a new FuelLaunchVisualizer with the given shooting algorithm and drivetrain. The
   * visualizer will create and launch projectiles with the parameters calculated by the shooting
   * algorithm when the launchProjectile command is executed, and will update the simulation and
   * logging for those projectiles when the updateSimulation and updateLogging methods are called,
   * respectively.
   *
   * @param shootingAlgorithm the shooting algorithm to use for calculating the launch parameters of
   *     the projectile
   * @param drivetrain the drivetrain to use for providing the current pose and velocity of the
   *     robot at launch time
   */
  public FuelLaunchVisualizer(ShootingAlgorithm shootingAlgorithm, Drive drivetrain) {
    super(
        () ->
            Fuel.launchTranslation(
                shootingAlgorithm.calculateToDoubleArray(
                    drivetrain.pose3d(), drivetrain.fieldRelativeChassisSpeeds()),
                drivetrain.pose3d()),
        () ->
            Fuel.launchVelocity(
                shootingAlgorithm.calculateToDoubleArray(
                    drivetrain.pose3d(), drivetrain.fieldRelativeChassisSpeeds()),
                drivetrain.pose3d(),
                drivetrain.fieldRelativeChassisSpeeds()),
        () ->
            Fuel.launchRotation(
                shootingAlgorithm.calculateToDoubleArray(
                    drivetrain.pose3d(), drivetrain.fieldRelativeChassisSpeeds()),
                drivetrain.pose3d()),
        Fuel::launchRotationalVelocity);
  }

  /**
   * Creates a new FuelLaunchVisualizer with the given launch parameters. The visualizer will create
   * and launch projectiles with the given parameters when the launchProjectile command is executed,
   * and will update the simulation and logging for those projectiles when the updateSimulation and
   * updateLogging methods are called, respectively.
   *
   * @param velocity a supplier that provides the velocity of the projectile at launch time
   * @param pitch a supplier that provides the pitch of the shooter at launch time
   * @param yaw a supplier that provides the yaw of the shooter at launch time
   * @param robotPose a supplier that provides the current pose of the robot at launch time
   * @param robotVelocity a supplier that provides the current velocity of the robot at launch time
   */
  public FuelLaunchVisualizer(
      DoubleSupplier velocity,
      DoubleSupplier pitch,
      DoubleSupplier yaw,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> Fuel.launchTranslation(pitch.getAsDouble(), yaw.getAsDouble(), robotPose.get()),
        () ->
            Fuel.launchVelocity(
                velocity.getAsDouble(),
                pitch.getAsDouble(),
                yaw.getAsDouble(),
                robotPose.get(),
                robotVelocity.get()),
        () -> Fuel.launchRotation(pitch.getAsDouble(), yaw.getAsDouble(), robotPose.get()),
        Fuel::launchRotationalVelocity);
  }

  @Override
  protected Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled) {
    return new Fuel().config(resolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
  }

  @Override
  public void updateSimulation() {
    Tracer.startTrace("Fuel Launcher Frame Generation");
    super.updateSimulation();
    Tracer.endTrace();
  }

  @Override
  public void updateLogging() {
    LoggingUtils.log("Fuel Visualizer/Scores", scores());
    LoggingUtils.log("Fuel Visualizer/Misses", misses());
    LoggingUtils.log("Fuel Visualizer/Fuel Poses", poses(), Pose3d.struct);
  }
}
