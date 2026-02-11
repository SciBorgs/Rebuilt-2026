package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public abstract class LaunchVisualizer {
  private int scores, misses;
  private boolean gravityEnabled, dragEnabled, torqueEnabled, liftEnabled;
  private final Supplier<double[]> launchTranslation,
      launchVelocity,
      launchRotation,
      launchRotationalVelocity;
  private final List<Projectile> projectiles = new ArrayList<>();

  private static final double COOLDOWN = 0.05;

  protected abstract Projectile createProjectile(
      boolean gravityEnabled, boolean dragEnabled, boolean torqueEnabled, boolean liftEnabled);

  public LaunchVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      Supplier<double[]> launchRotationalVelocity) {
    this.launchTranslation = launchTranslation;
    this.launchVelocity = launchVelocity;
    this.launchRotation = launchRotation;
    this.launchRotationalVelocity = launchRotationalVelocity;

    gravityEnabled = true;
    dragEnabled = true;
    torqueEnabled = true;
    liftEnabled = true;
  }

  public LaunchVisualizer config(boolean gravity, boolean drag, boolean torque, boolean lift) {
    gravityEnabled = gravity;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

  public Command launchProjectile() {
    return Commands.repeatingSequence(
        Commands.run(
                () -> {
                  Projectile projectile =
                      createProjectile(gravityEnabled, dragEnabled, torqueEnabled, liftEnabled);
                  projectiles.add(projectile);

                  projectile.launch(
                      launchTranslation.get(),
                      launchVelocity.get(),
                      launchRotation.get(),
                      launchRotationalVelocity.get());
                })
            .andThen(Commands.waitSeconds(COOLDOWN)));
  }

  public void periodic() {
    for (int index = 0; index < projectiles.size(); index++) {
      Projectile projectile = projectiles.get(index);

      if (projectile.checkIfMissed()) {
        misses++;
        projectiles.remove(index);
      }
      if (projectile.checkIfScored()) {
        scores++;
        projectiles.remove(index);
      }

      projectile.periodic();
    }

    logToNetworkTables();
  }

  protected abstract void logToNetworkTables();

  public Pose3d[] poses() {
    Pose3d[] poses = new Pose3d[projectiles.size()];
    for (int index = 0; index < poses.length; index++) poses[index] = projectiles.get(index).pose();
    return poses;
  }

  public int scores() {
    return scores;
  }

  public int misses() {
    return misses;
  }
}
