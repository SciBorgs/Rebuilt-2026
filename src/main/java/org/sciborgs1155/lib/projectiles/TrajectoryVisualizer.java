package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public abstract class TrajectoryVisualizer {
  private double airTime;
  private boolean scores, misses;
  private final Supplier<double[]> launchTranslation,
      launchVelocity,
      launchRotation,
      launchRotationalVelocity;

  protected abstract Projectile createProjectile(
      double resolution,
      boolean gravityEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled);

  public TrajectoryVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      Supplier<double[]> launchRotationalVelocity) {
    this.launchTranslation = launchTranslation;
    this.launchVelocity = launchVelocity;
    this.launchRotation = launchRotation;
    this.launchRotationalVelocity = launchRotationalVelocity;
  }

  public void periodic() {
    logToNetworkTables();
  }

  protected abstract void logToNetworkTables();

  public Pose3d[] trajectory(
      double resolution, boolean gravity, boolean drag, boolean torque, boolean lift) {
    int frames = 0;
    List<Pose3d> trajectory = new ArrayList<>();
    Projectile projectile = createProjectile(resolution, gravity, drag, torque, lift);

    projectile.launch(
        launchTranslation.get(),
        launchVelocity.get(),
        launchRotation.get(),
        launchRotationalVelocity.get());
    while (!projectile.checkIfMissed() && !projectile.checkIfScored()) {
      trajectory.add(projectile.pose());
      projectile.periodic();
      frames++;
    }

    misses = projectile.checkIfMissed();
    scores = projectile.checkIfScored();
    airTime = (double) frames / resolution;

    return trajectory.toArray(new Pose3d[0]);
  }

  public boolean scores() {
    return scores;
  }

  public boolean misses() {
    return misses;
  }

  public double airTime() {
    return airTime;
  }
}
