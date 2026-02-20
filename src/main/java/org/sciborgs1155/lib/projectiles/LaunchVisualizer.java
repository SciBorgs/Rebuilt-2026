package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;

/**
 * A class that manages the creation, simulation, and logging of simulated projectiles.
 *
 * @see Projectile
 */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public abstract class LaunchVisualizer {
  private int scores, misses;
  private boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;
  private final Supplier<double[]> launchTranslation, launchVelocity, launchRotation;

  private final DoubleSupplier launchRotationalVelocity;
  private final List<Projectile> projectiles = new ArrayList<>();
  private static final double COOLDOWN = 0.05;

  protected abstract Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled);

  /**
   * A class that manages the creation, simulation, and logging of simulated projectiles.
   *
   * @param launchTranslation a supplier that provides the translation of the projectile at launch
   *     time
   * @param launchVelocity a supplier that provides the velocity of the projectile at launch time
   * @param launchRotation a supplier that provides the rotation of the projectile at launch time
   * @param launchRotationalVelocity a supplier that provides the rotational velocity of the
   *     projectile at launch time
   */
  public LaunchVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    this.launchTranslation = launchTranslation;
    this.launchVelocity = launchVelocity;
    this.launchRotation = launchRotation;
    this.launchRotationalVelocity = launchRotationalVelocity;

    weightEnabled = true;
    dragEnabled = true;
    torqueEnabled = false;
    liftEnabled = false;
  }

  /**
   * Configures the visualizer's physics settings.
   *
   * @param weight Whether to apply weight to the projectile.
   * @param drag Whether to apply drag to the projectile.
   * @param torque Whether to apply torque to the projectile.
   * @param lift Whether to apply lift to the projectile.
   * @return this visualizer
   */
  public LaunchVisualizer config(boolean weight, boolean drag, boolean torque, boolean lift) {
    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

  /**
   * Returns a command that continuously launches projectiles with the given launch parameters and
   * physics settings.
   *
   * @return a command that continuously launches projectiles
   */
  public Command launchProjectile() {
    return Commands.repeatingSequence(
        Commands.runOnce(this::createAndLaunchProjectile).andThen(Commands.waitSeconds(COOLDOWN)));
  }

  private void createAndLaunchProjectile() {
    Projectile projectile =
        createProjectile(
            Projectile.RESOLUTION, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
    projectiles.add(projectile);

    projectile.launch(
        launchTranslation.get(),
        launchVelocity.get(),
        launchRotation.get(),
        launchRotationalVelocity.getAsDouble());
  }

  /** Updates the simulation for all projectiles in the visualizer. */
  public void updateSimulation() {
    for (int index = 0; index < projectiles.size(); index++) {
      Projectile projectile = projectiles.get(index);

      if (projectile.willMiss()) {
        misses++;
        projectiles.remove(index);
      }

      if (projectile.willScore()) {
        scores++;
        projectiles.remove(index);
      }

      projectile.periodic();
    }
  }

  /** Updates the logging for the visualizer's state. */
  public void updateLogging() {
    LoggingUtils.log("Launch Visualizer/Scores", scores());
    LoggingUtils.log("Launch Visualizer/Misses", misses());
    LoggingUtils.log("Launch Visualizer/Projectile Poses", poses(), Pose3d.struct);
  }

  /**
   * Returns the current poses of all projectiles in the visualizer.
   *
   * @return an array of Pose3d objects representing the current poses of all projectiles
   */
  public Pose3d[] poses() {
    Pose3d[] poses = new Pose3d[projectiles.size()];
    for (int index = 0; index < poses.length; index++) poses[index] = projectiles.get(index).pose();
    return poses;
  }

  /**
   * Returns the number of times a projectile has scored.
   *
   * @return the number of times a projectile has scored
   */
  public int scores() {
    return scores;
  }

  /**
   * Returns the number of times a projectile has missed.
   *
   * @return the number of times a projectile has missed
   */
  public int misses() {
    return misses;
  }
}
