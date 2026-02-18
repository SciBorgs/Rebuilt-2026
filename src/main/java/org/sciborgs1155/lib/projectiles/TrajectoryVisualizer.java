package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;

/**
 * A class that manages the creation, simulation, and logging of simulated projectile trajectories.
 *
 * @see Projectile
 */
public abstract class TrajectoryVisualizer {
  private double airTime;
  private static final int MAXIMUM_FRAMES = 250;

  @SuppressWarnings("PMD.OneDeclarationPerLine")
  private boolean scores, misses;
  @SuppressWarnings("PMD.OneDeclarationPerLine")
  private final Supplier<double[]> launchTranslation, launchVelocity, launchRotation;
  private final DoubleSupplier launchRotationalVelocity;

  protected abstract Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled);

  /**
   * Creates a new TrajectoryVisualizer with the given launch parameters. The visualizer will update
   * the trajectory when the updateLogging method is called.
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
  public TrajectoryVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    this.launchTranslation = launchTranslation;
    this.launchVelocity = launchVelocity;
    this.launchRotation = launchRotation;
    this.launchRotationalVelocity = launchRotationalVelocity;
  }

  /** Updates the logged trajectory in NetworkTables. */
  public void updateLogging() {
    LoggingUtils.log(
        "Trajectory Visualizer/Trajectory",
        trajectory(Projectile.RESOLUTION, true, true, true, true),
        Pose3d.struct);
    LoggingUtils.log("Trajectory Visualizer/Will score", willScore());
    LoggingUtils.log("Trajectory Visualizer/Will miss", willMiss());
    LoggingUtils.log("Trajectory Visualizer/Air Time", airTime());
  }

  /**
   * Simulates the projectile's trajectory with the given physics settings and returns an array of
   * Pose3d representing the projectile's pose at each step of the simulation. The simulation will
   * end when the projectile either scores or misses.
   *
   * @param resolution the resolution of the projectile's simulation, in steps per second
   * @param weight Whether to apply weight to the projectile.
   * @param drag Whether to apply drag to the projectile.
   * @param torque Whether to apply torque to the projectile.
   * @param lift Whether to apply lift to the projectile.
   * @return an array of Pose3d objects representing the projectile's trajectory
   */
  public Pose3d[] trajectory(
      double resolution, boolean weight, boolean drag, boolean torque, boolean lift) {
    int frames = 0;
    List<Pose3d> trajectory = new ArrayList<>();
    Projectile projectile = createProjectile(resolution, weight, drag, torque, lift);

    projectile.launch(
        launchTranslation.get(),
        launchVelocity.get(),
        launchRotation.get(),
        launchRotationalVelocity.getAsDouble());
    while (!projectile.willMiss() && !projectile.willScore()) {
      if (frames >= MAXIMUM_FRAMES) break;
      trajectory.add(projectile.pose());
      projectile.periodic();
      frames++;
    }

    misses = projectile.willMiss();
    scores = projectile.willScore();

    airTime = frames / resolution;

    return trajectory.toArray(new Pose3d[0]);
  }

  /**
   * Returns whether the projectile will score based on its current trajectory.
   *
   * @return true if the projectile will score, false otherwise
   */
  public boolean willScore() {
    return scores;
  }

  /**
   * Returns whether the projectile will miss based on its current trajectory.
   *
   * @return true if the projectile will miss, false otherwise
   */
  public boolean willMiss() {
    return misses;
  }

  /**
   * Returns the time the projectile will spend in the air based on its current trajectory.
   *
   * @return the time in seconds the projectile will spend in the air
   */
  public double airTime() {
    return airTime;
  }
}
