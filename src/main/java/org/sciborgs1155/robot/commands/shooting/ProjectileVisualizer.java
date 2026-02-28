package org.sciborgs1155.robot.commands.shooting;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tracer;

/**
 * A class that manages the creation, simulation, and logging of simulated projectiles.
 *
 * @see Projectile
 */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public abstract class ProjectileVisualizer {
  private double airTime, launchResolution, trajectoryResolution, cooldown;
  private int scores, misses;
  private boolean willScore, willMiss, launchEnabled, trajectoryEnabled;
  private boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;

  private Pose3d[] trajectory;
  private Pose3d initial;

  private final Supplier<double[]> launchTranslation, launchVelocity, launchRotation;
  private final DoubleSupplier launchRotationalVelocity;

  private final List<Projectile> projectiles = new ArrayList<>();

  private static final double DEFAULT_COOLDOWN = 0.05;
  private static final int MAX_TRAJECTORY_SIZE = 200;
  private static final double DEFAULT_LAUNCH_RESOLUTION = 80;
  private static final double DEFAULT_TRAJECTORY_RESOLUTION = 60;

  /** The delay between each update of the simulated projectile poses. */
  public static final double LAUNCH_PERIOD = 1 / DEFAULT_LAUNCH_RESOLUTION;

  /** The delay between each update of the simulated projectile trajectories. */
  public static final double TRAJECTORY_PERIOD = 1 / DEFAULT_TRAJECTORY_RESOLUTION;

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
  public ProjectileVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    this.launchTranslation = launchTranslation;
    this.launchVelocity = launchVelocity;
    this.launchRotation = launchRotation;
    this.launchRotationalVelocity = launchRotationalVelocity;

    trajectory = new Pose3d[0];

    launchResolution = DEFAULT_LAUNCH_RESOLUTION;
    trajectoryResolution = DEFAULT_TRAJECTORY_RESOLUTION;
    cooldown = DEFAULT_COOLDOWN;

    weightEnabled = true;
    dragEnabled = true;
    torqueEnabled = false;
    liftEnabled = false;

    launchEnabled = true;
    trajectoryEnabled = true;
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
  public ProjectileVisualizer configPhysics(
      boolean weight, boolean drag, boolean torque, boolean lift) {
    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

  /**
   * Configures the visualizer's generation settings.
   *
   * @param delay the minimum amount of time in between projectile launches
   * @param launch the resolution of the projectile's simulation, in steps per second
   * @param trajectory the resolution of the projectile's trajectory, in steps per second
   * @return this visualizer
   */
  public ProjectileVisualizer configGeneration(double delay, double launch, double trajectory) {
    cooldown = delay;
    launchResolution = launch;
    trajectoryResolution = trajectory;

    return this;
  }

  /**
   * Configures the visualizer's generation settings.
   *
   * @param projectiles whether or not to simulate projectiles being launched
   * @param trajectory whether or not to simulate the trajectory of the projectile
   * @return this visualizer
   */
  public ProjectileVisualizer config(boolean launch, boolean trajectory) {
    launchEnabled = launch;
    trajectoryEnabled = trajectory;

    return this;
  }

  /** Updates the logged trajectory in NetworkTables. */
  public void updateLogging() {
    LoggingUtils.log("Projectile Visualizer/Trajectory", trajectory(), Pose3d.struct);
    LoggingUtils.log("Projectile Visualizer/Will score", willScore());
    LoggingUtils.log("Projectile Visualizer/Will miss", willMiss());
    LoggingUtils.log("Projectile Visualizer/Air Time", airTime());
    LoggingUtils.log("Projectile Visualizer/Scores", scores());
    LoggingUtils.log("Projectile Visualizer/Misses", misses());
    LoggingUtils.log("Projectile Visualizer/Projectiles", poses(), Pose3d.struct);
    LoggingUtils.log("Projectile Visualizer/Launch pose", initial(), Pose3d.struct);
  }

  /**
   * Simulates the projectile's trajectory with the given physics settings and returns an array of
   * Pose3d representing the projectile's pose at each step of the simulation. The simulation will
   * end when the projectile either scores or misses.
   *
   * @param RESOLUTION the resolution of the projectile's simulation, in steps per second
   * @param weight Whether to apply weight to the projectile.
   * @param drag Whether to apply drag to the projectile.
   * @param torque Whether to apply torque to the projectile.
   * @param lift Whether to apply lift to the projectile.
   * @return an array of Pose3d objects representing the projectile's trajectory
   */
  public Pose3d[] generateTrajectory() {
    int frames = 0;
    List<Pose3d> trajectory = new ArrayList<>();
    Projectile projectile =
        createProjectile(
            trajectoryResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);

    projectile.launch(
        launchTranslation.get(),
        launchVelocity.get(),
        launchRotation.get(),
        launchRotationalVelocity.getAsDouble());

    initial = projectile.pose();
    while (!projectile.willMiss() && !projectile.willScore() && frames <= MAX_TRAJECTORY_SIZE) {
      trajectory.add(projectile.pose());
      projectile.periodic();
      frames++;
    }

    willMiss = projectile.willMiss();
    willScore = projectile.willScore();
    airTime = frames / trajectoryResolution;

    return trajectory.toArray(new Pose3d[0]);
  }

  /** Launches a singular projectile. */
  public void launchProjectile() {
    Projectile projectile =
        createProjectile(launchResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
    projectiles.add(projectile);
    projectile.launch(
        launchTranslation.get(),
        launchVelocity.get(),
        launchRotation.get(),
        launchRotationalVelocity.getAsDouble());
    initial = projectile.pose();
  }

  /**
   * Returns a command that continuously launches projectiles.
   *
   * @return a command that continuously launches projectiles
   */
  public Command launchProjectiles() {
    return Commands.repeatingSequence(
        Commands.runOnce(this::launchProjectile).andThen(Commands.waitSeconds(cooldown)));
  }

  /** Updates the simulation for all projectiles in the visualizer. */
  public void updateLaunchSimulation() {
    Tracer.startTrace("launch simulation");
    if (!launchEnabled) return;
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
    Tracer.endTrace();
  }

  /** Updates the displayed trajectory of the projectile. */
  public void updateTrajectorySimulation() {
    Tracer.startTrace("trajectory generation");
    trajectory = trajectoryEnabled ? generateTrajectory() : new Pose3d[0];
    Tracer.endTrace();
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
   * Returns whether the projectile will score based on its current trajectory.
   *
   * @return true if the projectile will score, false otherwise
   */
  public boolean willScore() {
    return willScore;
  }

  /**
   * Returns whether the projectile will miss based on its current trajectory.
   *
   * @return true if the projectile will miss, false otherwise
   */
  public boolean willMiss() {
    return willMiss;
  }

  /**
   * Returns the time the projectile will spend in the air based on its current trajectory.
   *
   * @return the time in seconds the projectile will spend in the air
   */
  public double airTime() {
    return airTime;
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

  /**
   * Returns the resolution of the projectile's simulation, in steps per second .
   *
   * @return the resolution of the projectile's simulation, in steps per second
   */
  public int resolution() {
    return misses;
  }

  /**
   * Returns the most recent generated trajectory of the projectile.
   *
   * @return the generated trajectory of the projectile
   */
  public Pose3d[] trajectory() {
    return trajectory.clone();
  }

  /**
   * The initial pose of the projectile
   *
   * @return the initial pose of the projectile.
   */
  public Pose3d initial() {
    return initial;
  }

  /** A class that models the physics of a projectile. */
  @SuppressWarnings("PMD.OneDeclarationPerLine")
  protected abstract static class Projectile {
    protected static final int X = 0, Y = 1, Z = 2;
    protected static final int ANGLE = 0, AXIS_X = 1, AXIS_Y = 2, AXIS_Z = 3;

    protected double resolution;
    protected boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;
    protected double[] translation, velocity, acceleration, rotation;
    protected double rotationalVelocity, rotationalAcceleration;

    /**
     * Returns the weight acceleration applied to the projectile based on its current trajectory.
     *
     * @return the weight acceleration applied to the projectile
     */
    protected abstract double[] weight();

    /**
     * Returns the drag acceleration applied to the projectile based on its current trajectory.
     *
     * @return the drag acceleration applied to the projectile
     */
    protected abstract double[] drag();

    /**
     * Returns the torque acceleration applied to the projectile based on its current trajectory.
     *
     * @return the torque acceleration applied to the projectile
     */
    protected abstract double torque();

    /**
     * Returns the lift acceleration applied to the projectile based on its current trajectory.
     *
     * @return the lift acceleration applied to the projectile
     */
    protected abstract double[] lift();

    /**
     * Returns whether the projectile will score based on its current trajectory.
     *
     * @return true if the projectile will score, false otherwise
     */
    protected abstract boolean willScore();

    /**
     * Returns whether the projectile will miss based on its current trajectory.
     *
     * @return true if the projectile will miss, false otherwise
     */
    protected abstract boolean willMiss();

    protected Projectile() {
      translation = new double[3];
      velocity = new double[3];
      acceleration = new double[3];

      rotation = new double[4];
      rotationalVelocity = 0;
      rotationalAcceleration = 0;

      weightEnabled = true;
      dragEnabled = true;
      torqueEnabled = true;
      liftEnabled = true;

      resolution = 50;
    }

    protected void launch(
        double[] launchTranslation,
        double[] launchVelocity,
        double[] launchRotation,
        double launchRotationalVelocity) {
      translation = launchTranslation.clone();
      velocity = launchVelocity.clone();
      acceleration = new double[3];

      rotation = launchRotation.clone();
      rotationalVelocity = launchRotationalVelocity;
      rotationalAcceleration = 0;
    }

    protected Projectile config(
        double fps, boolean weight, boolean drag, boolean torque, boolean lift) {
      resolution = fps;
      weightEnabled = weight;
      dragEnabled = drag;
      torqueEnabled = torque;
      liftEnabled = lift;

      return this;
    }

    protected void periodic() {
      translation[X] += velocity[X] / resolution;
      translation[Y] += velocity[Y] / resolution;
      translation[Z] += velocity[Z] / resolution;

      velocity[X] += acceleration[X] / resolution;
      velocity[Y] += acceleration[Y] / resolution;
      velocity[Z] += acceleration[Z] / resolution;

      acceleration[X] = 0;
      acceleration[Y] = 0;
      acceleration[Z] = 0;

      if (weightEnabled) {
        double[] weight = weight();

        acceleration[X] += weight[X];
        acceleration[Y] += weight[Y];
        acceleration[Z] += weight[Z];
      }

      if (dragEnabled) {
        double[] drag = drag();

        acceleration[X] += drag[X];
        acceleration[Y] += drag[Y];
        acceleration[Z] += drag[Z];
      }

      if (liftEnabled) {
        double[] lift = lift();

        acceleration[X] += lift[X];
        acceleration[Y] += lift[Y];
        acceleration[Z] += lift[Z];
      }

      rotation[ANGLE] += rotationalVelocity / resolution;
      rotationalVelocity += rotationalAcceleration / resolution;
      rotationalAcceleration = 0;

      if (torqueEnabled) {
        rotationalAcceleration = torque();
      }
    }

    protected Pose3d pose() {
      return new Pose3d(
          translation[X],
          translation[Y],
          translation[Z],
          new Rotation3d(
              VecBuilder.fill(rotation[AXIS_X], rotation[AXIS_Y], rotation[AXIS_Z]),
              rotation[ANGLE]));
    }

    protected void reset() {
      translation = new double[3];
      velocity = new double[3];
      acceleration = new double[3];

      rotation = new double[4];
      rotationalVelocity = 0;
      rotationalAcceleration = 0;
    }

    protected static double[] fromTranslation(Translation3d translation) {
      return new double[] {translation.getX(), translation.getY(), translation.getZ()};
    }

    protected static double[] toDirectionVector(double pitch, double yaw) {
      return new double[] {
        Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)
      };
    }

    protected static double[] rotateAroundZ(double[] vector, double angle) {
      return new double[] {
        vector[X] * Math.cos(angle) - vector[Y] * Math.sin(angle),
        vector[X] * Math.sin(angle) + vector[Y] * Math.cos(angle),
        vector[Z]
      };
    }

    protected static double[] add3(double[] vector1, double[] vector2) {
      return new double[] {
        vector1[X] + vector2[X], vector1[Y] + vector2[Y], vector1[Z] + vector2[Z]
      };
    }

    protected static double[] sub3(double[] vector1, double[] vector2) {
      return new double[] {
        vector1[X] - vector2[X], vector1[Y] - vector2[Y], vector1[Z] - vector2[Z]
      };
    }

    protected static double[] scale3(double[] vector, double scalar) {
      return new double[] {vector[X] * scalar, vector[Y] * scalar, vector[Z] * scalar};
    }

    protected static double[] scale4(double[] vector, double scalar) {
      return new double[] {
        vector[ANGLE] * scalar,
        vector[AXIS_X] * scalar,
        vector[AXIS_Y] * scalar,
        vector[AXIS_Z] * scalar
      };
    }

    protected static double norm3(double[] vector) {
      return Math.sqrt(vector[X] * vector[X] + vector[Y] * vector[Y] + vector[Z] * vector[Z]);
    }
  }
}
