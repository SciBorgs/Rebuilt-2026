package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;

/**
 * A class that manages the creation, simulation, and logging of simulated FUEL trajectories.
 *
 * @see Fuel
 */
public class FuelTrajectoryVisualizer extends TrajectoryVisualizer {
  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL trajectories.
   *
   * @param launchVelocity a supplier that provides the velocity of the projectile at launch time
   * @param robotPose a supplier that provides the pose of the robot at launch time
   * @param robotVelocity a supplier that provides the velocity of the robot at launch time
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
        trajectory(Projectile.RESOLUTION, true, true, false, false),
        Pose3d.struct);
    Tracer.endTrace();
  }
}
