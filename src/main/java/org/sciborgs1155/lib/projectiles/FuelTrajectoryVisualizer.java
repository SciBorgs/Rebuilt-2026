package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.lib.shooting.ShootingAlgorithm;
import org.sciborgs1155.robot.drive.Drive;

/**
 * A class that manages the creation, simulation, and logging of simulated FUEL trajectories.
 *
 * @see Projectile
 */
public class FuelTrajectoryVisualizer extends TrajectoryVisualizer {
  private final DoubleEntry resolutionEntry =
      Tuning.entry("Fuel Visualizer/Resolution", Projectile.RESOLUTION);

  /**
   * Creates a new FuelTrajectoryVisualizer with the given launch parameters. The visualizer will
   * create and log the trajectory of the projectile with the given launch parameters.
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
  public FuelTrajectoryVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    super(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);
  }

  /**
   * Creates a new FuelTrajectoryVisualizer with the given launch parameters. The visualizer will
   * create and log the trajectory of the projectile with the given launch parameters.
   *
   * @param launchVelocity a supplier that provides the current velocity of the projectile at launch
   *     time
   * @param robotPose a supplier that provides the current pose of the robot at launch time
   * @param robotVelocity a supplier that provides the current velocity of the robot at launch time
   */
  public FuelTrajectoryVisualizer(
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
   * Creates a new FuelTrajectoryVisualizer with the given launch parameters. The visualizer will
   * create and log the trajectory of the projectile with the given launch parameters.
   *
   * @param shootingAlgorithm the shooting algorithm to use for calculating the launch parameters of
   *     the projectile
   * @param drivetrain the drivetrain to use for providing the current pose and velocity of the
   *     robot at launch time
   */
  public FuelTrajectoryVisualizer(ShootingAlgorithm shootingAlgorithm, Drive drivetrain) {
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
   * Creates a new FuelTrajectoryVisualizer with the given launch parameters. The visualizer will
   * create and log the trajectory of the projectile with the given launch parameters.
   *
   * @param pitch a supplier that provides the pitch of the shooter at launch time
   * @param yaw a supplier that provides the yaw of the shooter at launch time
   * @param robotPose a supplier that provides the current pose of the robot at launch time
   * @param robotVelocity a supplier that provides the current velocity of the robot at launch time
   */
  public FuelTrajectoryVisualizer(
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
  public void updateLogging() {
    Tracer.startTrace("Fuel Trajectory Generation");
    LoggingUtils.log("Fuel Visualizer/Score", willScore());
    LoggingUtils.log("Fuel Visualizer/Miss", willMiss());
    LoggingUtils.log("Fuel Visualizer/Air Time", airTime());
    LoggingUtils.log(
        "Fuel Visualizer/Trajectory",
        trajectory(resolutionEntry.get(), true, true, false, false),
        Pose3d.struct);
    Tracer.endTrace();
  }
}
