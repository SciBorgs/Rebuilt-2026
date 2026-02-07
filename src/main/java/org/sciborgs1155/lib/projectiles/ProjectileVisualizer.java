package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

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
  protected static final double COOLDOWN = 1;

  /** The viscosity of the air (METER^2 / SECOND) */
  protected static final double AIR_VISCOSITY = 0.0000151;

  /** When deleting projectiles, their points are stored here. */
  private int deletedScores;

  /** When deleting projectiles, their misses are stored here. */
  private int deletedMisses;

  /** The index of the latest projectile to have been launched. */
  private int projectileIndex;

  /** All projectiles currently being simulated. */
  private final List<Projectile> projectiles;

  /** A supplier for the pose of the robot (METERS). */
  protected final Supplier<Pose3d> robotPose;

  /** A supplier for the pose of the robot (METERS). */
  protected final Supplier<ChassisSpeeds> robotVelocity;

  /**
   * Calculates the launch translation of the projectile.
   *
   * @param robotPose The current pose of the robot.
   * @return The field-relative launch translation of the projectile (METERS).
   */
  protected abstract Vector<N3> launchTranslation(Pose3d robotPose);

  /**
   * Calculates the launch velocity of the projectile.
   *
   * @param robotPose The current pose of the robot (METERS). Necessary for calculation regarding
   *     rotational velocity.
   * @param robotVelocity The current velocity of the robot.
   * @return The field-relative launch velocity of the projectile (METERS / SECOND).
   */
  protected abstract Vector<N3> launchVelocity(Pose3d robotPose, ChassisSpeeds robotVelocity);

  /**
   * Calculates the launch rotation of the projectile (YAW AND PITCH).
   *
   * @param robotPose The current pose of the robot (METERS).
   * @return The projectile-relative launch rotation of the projectile (RADIANS).
   */
  protected abstract Vector<N2> launchRotation(Pose3d robotPose);

  /**
   * Calculates the launch rotational velocity of the projectile (YAW AND PITCH).
   *
   * @param robotPose The current pose of the robot (METERS).
   * @return The projectile-relative launch rotational velocity of the projectile (RADIANS /
   *     SECOND).
   */
  protected abstract Vector<N2> launchRotationalVelocity(Pose3d robotPose);

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
    projectiles = new ArrayList<>(1);
  }

  /** Publishes the projectile display data to {@code NetworkTables}. */
  public void periodic() {
    Tracer.startTrace("Projectile Visualizer");

    int scores = deletedScores;
    int misses = deletedMisses;

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

    LoggingUtils.log("Projectile Visualizer/Scores", scores);
    LoggingUtils.log("Projectile Visualizer/Misses", misses);
    LoggingUtils.log("Projectile Visualizer/Projectile Poses", projectilePoses(), Pose3d.struct);
  }

  /**
   * Launches projectiles from the robot for the duration of the command.
   *
   * @return A command to launch projectiles.
   */
  public Command launchProjectile() {
    return Commands.deferredProxy(() -> Commands.runOnce(this::launch))
        .andThen(Commands.waitSeconds(COOLDOWN))
        .withName("LAUNCH PROJECTILE");
  }

  /** Launches a single projectile from the robot. */
  private void launch() {
    getLaunchableProjectile()
        .launch(
            launchTranslation(robotPose.get().plus(CENTER_TO_SHOOTER)),
            launchVelocity(robotPose.get(), robotVelocity.get()),
            launchRotation(robotPose.get()),
            launchRotationalVelocity(robotPose.get()));
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
