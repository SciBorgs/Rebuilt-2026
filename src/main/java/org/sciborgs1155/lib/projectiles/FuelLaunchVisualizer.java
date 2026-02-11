package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.lib.shooting.ShootingAlgorithm;
import org.sciborgs1155.robot.drive.Drive;

public class FuelLaunchVisualizer extends LaunchVisualizer {
  public FuelLaunchVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      Supplier<double[]> launchRotationalVelocity) {
    super(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);
  }

  public FuelLaunchVisualizer(
      Supplier<double[]> launchVelocity,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> Fuel.launchTranslation(launchVelocity.get(), robotPose.get()),
        () -> Fuel.launchVelocity(launchVelocity.get(), robotPose.get(), robotVelocity.get()),
        () -> Fuel.launchRotation(robotPose.get()),
        () -> Fuel.launchRotationalVelocity(robotPose.get()));
  }

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
        () -> Fuel.launchRotation(drivetrain.pose3d()),
        () -> Fuel.launchRotationalVelocity(drivetrain.pose3d()));
  }

  public FuelLaunchVisualizer(
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
  public void periodic() {
    Tracer.startTrace("Fuel Launcher Frame Generation");
    super.periodic();
    Tracer.endTrace();
  }

  @Override
  protected void logToNetworkTables() {
    LoggingUtils.log("Fuel Visualizer/Scores", scores());
    LoggingUtils.log("Fuel Visualizer/Misses", misses());
    LoggingUtils.log("Fuel Visualizer/Fuel Poses", poses(), Pose3d.struct);
  }
}
