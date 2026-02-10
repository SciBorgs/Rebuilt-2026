package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;

public class FuelTrajectoryVisualizer extends TrajectoryVisualizer {
  public FuelTrajectoryVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      Supplier<double[]> launchRotationalVelocity) {
    super(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);
  }

  public FuelTrajectoryVisualizer(
      Supplier<double[]> launchVelocity,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> Fuel.launchTranslation(launchVelocity.get(), robotPose.get()),
        () -> Fuel.launchVelocity(launchVelocity.get(), robotPose.get(), robotVelocity.get()),
        () -> Fuel.launchRotation(robotPose.get()),
        () -> Fuel.launchRotationalVelocity(robotPose.get()));
  }

  public FuelTrajectoryVisualizer(
      DoubleSupplier wheelVelocity,
      DoubleSupplier pitch,
      DoubleSupplier yaw,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> Fuel.launchTranslation(pitch.getAsDouble(), yaw.getAsDouble(), robotPose.get()),
        () ->
            Fuel.launchVelocity(
                wheelVelocity.getAsDouble(),
                pitch.getAsDouble(),
                yaw.getAsDouble(),
                robotPose.get(),
                robotVelocity.get()),
        () -> Fuel.launchRotation(robotPose.get()),
        () -> Fuel.launchRotationalVelocity(robotPose.get()));
  }

  @Override
  protected Projectile createProjectile(
      boolean gravityEnabled, boolean dragEnabled, boolean torqueEnabled, boolean liftEnabled) {
    return new Fuel().config(gravityEnabled, dragEnabled, torqueEnabled, liftEnabled);
  }

  @Override
  protected void logToNetworkTables() {
    Tracer.startTrace("Fuel Trajectory Generation");
    LoggingUtils.log("Fuel Visualizer/Score", scores());
    LoggingUtils.log("Fuel Visualizer/Miss", misses());
    LoggingUtils.log(
        "Fuel Visualizer/Trajectory", trajectory(true, false, false, false), Pose3d.struct);
    Tracer.endTrace();
  }
}
