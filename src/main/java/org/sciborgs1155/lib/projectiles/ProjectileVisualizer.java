package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;

/** Simulates the behavior of multiple {@code Projectiles}. */
public abstract class ProjectileVisualizer {
  /** Acceleration due to gravity (METERS / SECOND^2). */
  protected static final double GRAVITY = -9.81;

  /** The mass density of the air (KILOGRAMS / METER^3). */
  protected static final double AIR_DENSITY = 1.225;

  /** How long each frame of the animation is (SECONDS / FRAME). */
  protected static final double FRAME_LENGTH = 0.02;

  /** How fast the visualizer can launch projectiles (SECONDS / LAUNCH). */
  protected static final double COOLDOWN = 0.05;

  /** The amount of times a projectile has been scored. */
  private int scores;

  /** The amount of times a projectile has not scored. */
  private int misses;

  /** When deleting projectiles, their points are stored here. */
  private int deletedScores;

  /** When deleting projectiles, their misses are stored here. */
  private int deletedMisses;

  /** The index of the latest projectile to have been launched. */
  private int projectileIndex;

  /** All projectiles currently being simulated. */
  private final List<Projectile> projectiles;

  /** A supplier for the current pose of the shooter (METERS). */
  private final Supplier<Vector<N3>> projectileLaunchTranslation;

  /** A supplier for the launch velocity of the projectile (METERS PER SECOND). */
  private final Supplier<Vector<N3>> projectileLaunchVelocity;

  /** A supplier for the pose of the robot (METERS). */
  protected final Supplier<Pose3d> robotPose;

  /** A supplier for the pose of the robot (METERS). */
  protected final Supplier<ChassisSpeeds> robotVelocity;

  /** The amount of times a projectile has been scored. */
  protected int scores() {
    return scores;
  }

  /** The amount of times a projectile has not scored. */
  protected int misses() {
    return misses;
  }

  /**
   * Calculates the launch speed of the projectile.
   *
   * @return The launch speed of the projectile (METERS PER SECOND).
   */
  protected abstract double launchSpeed();

  /**
   * Calculates the launch translation of the projectile.
   *
   * @param robotPose The current pose of the robot.
   * @return The field-relative launch translation of the projectile (METERS).
   */
  protected abstract Vector<N3> launchTranslation(Pose3d robotPose);

  /**
   * Calculates the launch direction of the projectile.
   *
   * @param robotPose The current pose of the robot (METERS).
   * @return The field-relative launch direction of the projectile (X, Y, and Z UNIT VECTOR).
   */
  protected abstract Vector<N3> launchDirection(Pose3d robotPose);

  /**
   * Calculates the velocity of the launch translation on the field (accounting for the velocity of
   * the robot).
   *
   * @param robotPose The current pose of the robot (METERS). Necessary for calculation regarding
   *     rotational velocity.
   * @param robotVelocity The current velocity of the robot.
   * @return The field-relative velocity of the launch translation origin (METERS PER SECOND).
   */
  protected abstract Vector<N3> launcherVelocity(Pose3d robotPose, ChassisSpeeds robotVelocity);

  /**
   * Creates and returns a new projectile instance.
   *
   * @return A projectile.
   */
  protected abstract Projectile createProjectile();

  /**
   * Simulates the behavior of multiple {@code Projectiles}. Parameters used to calculate projectile
   * trajectory after launch.
   *
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
   */
  public ProjectileVisualizer(
      Supplier<Pose3d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
    robotPose = robotPoseSupplier;
    robotVelocity = robotVelocitySupplier;
    projectileLaunchVelocity =
        () ->
            launchDirection(robotPose.get())
                .times(launchSpeed())
                .plus(launcherVelocity(robotPose.get(), robotVelocity.get()));
    projectileLaunchTranslation = () -> launchTranslation(robotPose.get());
    projectiles = new ArrayList<>(1);
  }

  /**
   * Launches a single projectile from the robot.
   *
   * @return A command to launch a projectile.
   */
  public Command launchProjectile() {
    return Commands.deferredProxy(
            () ->
                Commands.runOnce(
                    () ->
                        getLaunchableProjectile()
                            .launch(
                                projectileLaunchTranslation.get(), projectileLaunchVelocity.get())))
        .andThen(Commands.waitSeconds(COOLDOWN))
        .withName("LAUNCH PROJECTILE");
  }

  /** Publishes the projectile display data to {@code NetworkTables}. */
  public void periodic() {
    Tracer.startTrace("Projectile Visualizer");

    scores = deletedScores;
    misses = deletedMisses;

    for (int index = 0; index < projectiles.size(); index++) {
      // UPDATING SIMULATIONS
      projectiles.get(index).nextFrame();

      // UPDATE SCORES / MISSES
      scores += projectiles.get(index).scores();
      misses += projectiles.get(index).misses();

      // DELETE IDLE PROJECTILES
      if (projectiles.get(index).hasBeenLaunched()) {
        deletedScores += projectiles.get(index).scores();
        deletedMisses += projectiles.get(index).misses();
        projectiles.remove(index);
      }
    }

    Tracer.endTrace();

    LoggingUtils.log("Projectile Visualizer/Scores", scores());
    LoggingUtils.log("Projectile Visualizer/Misses", misses());
    LoggingUtils.log("Projectile Visualizer/Projectile Poses", projectilePoses(), Pose3d.struct);
  }

  /**
   * Returns the first unlaunched projectile. If all projectiles have been launched, create a new
   * projectile.
   *
   * @return The first launchable projectile.
   */
  protected Projectile getLaunchableProjectile() {
    // RESET INDEX IF OUT OF BOUNDS
    if (projectileIndex >= projectiles.size() - 1) projectileIndex = 0;

    // WHETHER OR NOT THE ENTIRE ARRAY IS GOING TO BE ITERATED THROUGH
    boolean fullCycle = projectileIndex == 0;

    // ITERATE THROUGH NON-CYCLED PROJECTILE
    for (projectileIndex++; projectileIndex < projectiles.size(); projectileIndex++)
      if (!projectiles.get(projectileIndex).isBeingLaunched())
        return projectiles.get(projectileIndex);

    // ITERATE THROUGH REST OF THE PROJECTILES
    if (!fullCycle) {
      projectileIndex = 0;
      return getLaunchableProjectile();
    }

    // IF ITERATED THROUGH WHOLE ARRAY, CREATE NEW PROJECTILE
    Projectile projectile = createProjectile();
    projectiles.add(projectile);

    return projectile;
  }

  /**
   * Returns poses of every projectile currently being simulated.
   *
   * @return The poses of every projectile currently being simulated (METERS).
   */
  protected Pose3d[] projectilePoses() {
    Pose3d[] projectilePoses = new Pose3d[projectiles.size()];
    for (int index = 0; index < projectilePoses.length; index++)
      projectilePoses[index] = projectiles.get(index).pose();
    return projectilePoses;
  }
}
