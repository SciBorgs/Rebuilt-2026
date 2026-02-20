package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;

/**
 * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
 *
 * @see Fuel
 */
public class FuelLaunchVisualizer extends LaunchVisualizer {

  /**
   * A class that manages the creation, simulation, and logging of simulated FUEL projectiles.
   *
   * @param launchVelocity a supplier that provides the velocity of the projectile at launch time
   * @param robotPose a supplier that provides the pose of the robot at launch time
   * @param robotVelocity a supplier that provides the velocity of the robot at launch time
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
