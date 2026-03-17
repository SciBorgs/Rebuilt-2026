package org.sciborgs1155.robot.commands.shooting;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;

@SuppressWarnings({
  "PMD.GodClass",
  "PMD.AvoidUsingVolatile",
  "PMD.TooManyFields",
  "PMD.OneDeclarationPerLine",
  "PMD.AvoidSynchronizedStatement",
  "PMD.AvoidLiteralsInIfCondition"
})
public abstract class ProjectileVisualizer {
  /** Tolerance used to detect input changes. */
  protected static final double EPS = 1e-6;

  private double airTime;
  private int scores, misses;
  private boolean willScore, willMiss;

  private boolean launchEnabled = true;
  private boolean trajectoryEnabled = true;

  private boolean weightEnabled = true;
  private boolean dragEnabled = true;
  private boolean torqueEnabled = true;
  private boolean liftEnabled = true;

  private double launchResolution = 100;
  private double trajectoryResolution = 100;
  private double cooldown = 0.05;
  private double maxAirTime = 3.0;

  private int maxLaunchFrames, maxTrajectoryFrames;
  private double launchDt, trajectoryDt;
  private Pose3d initial, ending;

  private volatile Pose3d[] trajectory = new Pose3d[0];

  private final Supplier<double[]> initialTranslation;
  private final Supplier<double[]> initialVelocity;
  private final Supplier<double[]> initialRotation;
  private final DoubleSupplier initialRotationalVelocity;

  private final List<Projectile> projectiles = new ArrayList<>();

  /** Object pool for projectiles. */
  private final Deque<Projectile> projectilePool = new ArrayDeque<>();

  /** Reusable trajectory buffer. */
  private final List<Pose3d> trajectoryBuffer = new ArrayList<>(512);

  private final double[] lastInitialTranslation = new double[3];
  private final double[] lastInitialVelocity = new double[3];
  private final double[] lastInitialRotation = new double[3];
  private double lastInitialRotationalVelocity;

  private boolean launchStateInitialized;
  private volatile boolean trajectoryDirty = true;

  private final ScheduledExecutorService executor =
      Executors.newScheduledThreadPool(
          2,
          runnable -> {
            Thread thread = new Thread(runnable);
            thread.setName("Projectile Visualizer");
            return thread;
          });

  private final AtomicBoolean running = new AtomicBoolean(false);

  protected abstract Projectile createProjectile();

  protected ProjectileVisualizer(
      Supplier<double[]> initialTranslation,
      Supplier<double[]> initialVelocity,
      Supplier<double[]> initialRotation,
      DoubleSupplier initialRotationalVelocity) {
    this.initialTranslation = initialTranslation;
    this.initialVelocity = initialVelocity;
    this.initialRotation = initialRotation;
    this.initialRotationalVelocity = initialRotationalVelocity;

    recomputeFrameLimits();
  }

  private void recomputeFrameLimits() {
    maxLaunchFrames = (int) (maxAirTime * launchResolution);
    maxTrajectoryFrames = (int) (maxAirTime * trajectoryResolution);

    launchDt = 1.0 / launchResolution;
    trajectoryDt = 1.0 / trajectoryResolution;
  }

  protected static boolean diff(double number1, double number2) {
    return Math.abs(number1 - number2) > EPS;
  }

  protected static boolean diff(double[] vector1, double[] vector2) {
    return diff(vector1[Projectile.X], vector2[Projectile.X])
        || diff(vector1[Projectile.Y], vector2[Projectile.Y])
        || diff(vector1[Projectile.Z], vector2[Projectile.Z]);
  }

  protected static void validate(double[] vector) {
    if (vector == null) System.arraycopy(new double[3], 0, vector, 0, 3);
    if (vector.length < 3) System.arraycopy(new double[3], 0, vector, 0, 3);

    double x = vector[Projectile.X];
    double y = vector[Projectile.Y];
    double z = vector[Projectile.Z];

    if (x != x) vector[Projectile.X] = 0;
    if (y != y) vector[Projectile.Y] = 0;
    if (z != z) vector[Projectile.Z] = 0;
  }

  private void checkLaunchState() {
    double[] translation = initialTranslation.get();
    double[] velocity = initialVelocity.get();
    double[] rotation = initialRotation.get();
    double rotationalVelocity = initialRotationalVelocity.getAsDouble();

    boolean changed = false;
    if (launchStateInitialized) {
      if (diff(translation, lastInitialTranslation)
          || diff(velocity, lastInitialVelocity)
          || diff(rotation, lastInitialRotation)
          || diff(rotationalVelocity, lastInitialRotationalVelocity)) changed = true;
    } else {
      changed = true;
      launchStateInitialized = true;
    }

    if (changed) {
      System.arraycopy(translation, 0, lastInitialTranslation, 0, 3);
      System.arraycopy(velocity, 0, lastInitialVelocity, 0, 3);
      System.arraycopy(rotation, 0, lastInitialRotation, 0, 3);

      lastInitialRotationalVelocity = rotationalVelocity;
      trajectoryDirty = true;
    }
  }

  private Projectile obtainProjectile() {
    Projectile projectile = projectilePool.pollFirst();
    if (projectile == null) projectile = createProjectile();

    projectile.config(launchResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);

    return projectile;
  }

  private void recycleProjectile(Projectile projectile) {
    projectile.reset();
    projectilePool.addFirst(projectile);
  }

  /**
   * Starts the simulation thread. Note that once the thread has been started, no further changes
   * can be made to the simulation resolutions.
   */
  public void startSimulation() {
    if (!running.compareAndSet(false, true)) return;

    executor.scheduleAtFixedRate(
        this::updateLaunchSimulation, 0, (long) (launchDt * 1000000), TimeUnit.MICROSECONDS);

    executor.scheduleAtFixedRate(
        this::updateTrajectorySimulation,
        0,
        (long) (trajectoryDt * 1000000),
        TimeUnit.MICROSECONDS);
  }

  /**
   * Ends the simulation thread. Note that once the thread has been ended, it may not be started
   * again.
   */
  public void endSimulation() {
    running.set(false);
  }

  /**
   * Configures physics settings.
   *
   * @param weight whether or not to simulate gravity
   * @param drag whether or not to simulate drag
   * @param torque whether or not to simulate torque
   * @param lift whether or not to simulate lift (magnus force)
   */
  public ProjectileVisualizer configPhysics(
      boolean weight, boolean drag, boolean torque, boolean lift) {
    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    trajectoryDirty = true;

    return this;
  }

  /**
   * Configures generation settings.
   *
   * @param delay the minimum delay between projectile launches. Requires launch mode to be enabled
   * @param air the maximum amount of time a projectile can spend in air before being deleted
   *     (seconds)
   * @param launch the resolution of the launched projectiles (steps per second). Requires launch
   *     mode to be enabled. Note that once the thread has been started, no further changes can be
   *     made to the simulation resolutions.
   * @param trajectory the resolution of the trajectory (steps per second). Requires trajectory mode
   *     to be enabled. Note that once the thread has been started, no further changes can be made
   *     to the simulation resolutions.
   */
  public ProjectileVisualizer configGeneration(
      double delay, double air, double launch, double trajectory) {
    cooldown = delay;
    maxAirTime = air;

    launchResolution = launch;
    trajectoryResolution = trajectory;

    recomputeFrameLimits();
    trajectoryDirty = true;

    return this;
  }

  /**
   * Configures whether or not launch mode/trajectory mode is enabled. Both can be enabled
   * simultaneously.
   *
   * @param launch Launch mode allows for the launching of individual projectiles in real time
   * @param trajectory Trajectory mode allows for the generation of a complete trajectory ahead of
   *     time (x50 more resource intensive)
   */
  public ProjectileVisualizer config(boolean launch, boolean trajectory) {
    launchEnabled = launch;
    trajectoryEnabled = trajectory;

    return this;
  }

  private void launchProjectile() {
    if (!running.get()) return;
    Projectile projectile = obtainProjectile();

    projectile.initialize(
        initialTranslation.get(),
        initialVelocity.get(),
        initialRotation.get(),
        initialRotationalVelocity.getAsDouble());

    synchronized (projectiles) {
      projectiles.add(projectile);
    }

    initial = projectile.pose();
  }

  /** A command to launch projectiles continuously with a set delay. */
  public Command launchProjectiles() {
    return Commands.repeatingSequence(
        Commands.runOnce(this::launchProjectile).andThen(Commands.waitSeconds(cooldown)));
  }

  private void updateLaunchSimulation() {
    if (!launchEnabled) return;

    synchronized (projectiles) {
      for (int index = projectiles.size() - 1; index >= 0; index--) {
        Projectile projectile = projectiles.get(index);

        if (projectile.willMiss()) {
          misses++;
          ending = projectile.pose();
          projectiles.remove(index);
          recycleProjectile(projectile);
          continue;
        }

        if (projectile.willScore()) {
          scores++;
          ending = projectile.pose();
          projectiles.remove(index);
          recycleProjectile(projectile);
          continue;
        }

        if (projectile.frames >= maxLaunchFrames) {
          ending = projectile.pose();
          projectiles.remove(index);
          recycleProjectile(projectile);
          continue;
        }

        projectile.step();
      }
    }
  }

  private void updateTrajectorySimulation() {
    checkLaunchState();
    if (!trajectoryEnabled || !trajectoryDirty) return;

    trajectory = generateTrajectory();
    trajectoryDirty = false;
  }

  private Pose3d[] generateTrajectory() {
    trajectoryBuffer.clear();
    Projectile projectile = obtainProjectile();
    projectile.config(trajectoryResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);

    projectile.initialize(
        initialTranslation.get(),
        initialVelocity.get(),
        initialRotation.get(),
        initialRotationalVelocity.getAsDouble());

    initial = projectile.pose();

    int frames = 0;
    while (!projectile.willMiss() && !projectile.willScore() && frames <= maxTrajectoryFrames) {
      trajectoryBuffer.add(projectile.pose());
      projectile.step();
      frames++;
    }

    ending = projectile.pose();
    willMiss = projectile.willMiss();
    willScore = projectile.willScore();
    airTime = frames / trajectoryResolution;

    recycleProjectile(projectile);
    return trajectoryBuffer.toArray(new Pose3d[0]);
  }

  /** The poses of the projectiles being simulated. Requires launch mode to be enabled. */
  public Pose3d[] poses() {
    synchronized (projectiles) {
      Pose3d[] poses = new Pose3d[projectiles.size()];

      for (int index = 0; index < poses.length; index++)
        poses[index] = projectiles.get(index).pose();

      return poses;
    }
  }

  /**
   * The trajectory of the projectile when launched with the current parameters. Requires trajectory
   * to be enabled.
   */
  public Pose3d[] trajectory() {
    Pose3d[] output = new Pose3d[trajectory.length];
    System.arraycopy(trajectory, 0, output, 0, trajectory.length);
    return output;
  }

  /** The initial pose of the projectile. */
  public Pose3d initial() {
    return initial;
  }

  /** The final pose of the projectile. */
  public Pose3d ending() {
    return ending;
  }

  /**
   * Whether or not the projectile will hit the goal when launched with the current parameters.
   * Requires trajectory to be enabled.
   */
  public boolean willScore() {
    return willScore;
  }

  /**
   * Whether or not the projectile will miss the goal when launched with the current parameters.
   * Requires trajectory to be enabled.
   */
  public boolean willMiss() {
    return willMiss;
  }

  /**
   * The amount of time the projectile spends in the air (seconds). Requires trajectory to be
   * enabled.
   */
  public double airTime() {
    return airTime;
  }

  /**
   * The amount of time a projectile has been launched and scored into the goal. Requires launch to
   * be enabled.
   */
  public int scores() {
    return scores;
  }

  /**
   * The amount of time a projectile has been launched and missed the goal. Requires launch to be
   * enabled.
   */
  public int misses() {
    return misses;
  }

  /** Logs visualizer data to NetworkTables. */
  public void updateLogging() {
    LoggingUtils.log("Projectile Visualizer/Trajectory", trajectory(), Pose3d.struct);
    LoggingUtils.log("Projectile Visualizer/Will score", willScore);
    LoggingUtils.log("Projectile Visualizer/Will miss", willMiss);
    LoggingUtils.log("Projectile Visualizer/Air Time", airTime);
    LoggingUtils.log("Projectile Visualizer/Scores", scores);
    LoggingUtils.log("Projectile Visualizer/Misses", misses);
    LoggingUtils.log("Projectile Visualizer/Projectiles", poses(), Pose3d.struct);
    LoggingUtils.log("Projectile Visualizer/Launch pose", initial, Pose3d.struct);
    LoggingUtils.log("Projectile Visualizer/Ending pose", ending, Pose3d.struct);
  }

  protected abstract static class Projectile {
    protected static final double GRAVITY = -9.80665;
    protected static final double AIR_DENSITY = 1.225;

    protected static final int X = 0, Y = 1, Z = 2;

    protected int frames;

    protected double resolution, delta;
    protected boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;
    protected double x, y, z;
    protected double vx, vy, vz;
    protected double ax, ay, az;
    protected double roll, pitch, yaw;
    protected double omega;
    protected double alpha;

    protected double weight() {
      return GRAVITY;
    }

    protected abstract double[] drag();

    protected abstract double[] lift();

    protected abstract double torque();

    protected abstract boolean willScore();

    protected abstract boolean willMiss();

    protected Projectile config(
        double fps, boolean weight, boolean drag, boolean torque, boolean lift) {
      resolution = fps;
      delta = 1.0 / fps;

      weightEnabled = weight;
      dragEnabled = drag;
      torqueEnabled = torque;
      liftEnabled = lift;

      return this;
    }

    protected void initialize(
        double[] initialTranslation,
        double[] initialVelocity,
        double[] initialRotation,
        double initialRotationalVelocity) {
      // INPUT VALIDATION
      validate(initialTranslation);
      validate(initialVelocity);
      validate(initialRotation);

      x = initialTranslation[X];
      y = initialTranslation[Y];
      z = initialTranslation[Z];

      vx = initialVelocity[X];
      vy = initialVelocity[Y];
      vz = initialVelocity[Z];

      ax = 0;
      ay = 0;
      az = 0;

      roll = initialRotation[X];
      pitch = initialRotation[Y];
      yaw = initialRotation[Z];

      // INPUT VALIDATION
      omega =
          initialRotationalVelocity == initialRotationalVelocity ? initialRotationalVelocity : 0;
      alpha = 0;

      frames = 0;
    }

    protected void step() {
      // INTEGRATE POSITION
      x += vx * delta;
      y += vy * delta;
      z += vz * delta;

      // INTEGRATE VELOCITY
      vx += ax * delta;
      vy += ay * delta;
      vz += az * delta;

      // RESET ACCELERATION
      ax = 0;
      ay = 0;
      az = 0;

      // WEIGHT
      if (weightEnabled) az += GRAVITY;

      // DRAG
      if (dragEnabled) {
        double[] drag = drag();
        ax += drag[X];
        ay += drag[Y];
        az += drag[Z];
      }

      // LIFT
      if (liftEnabled) {
        double[] lift = lift();
        ax += lift[X];
        ay += lift[Y];
        az += lift[Z];
      }

      // INTEGRATE ROTATION
      pitch += omega * delta;
      omega += alpha * delta;
      alpha = torqueEnabled ? torque() : 0;

      frames++;
    }

    protected Pose3d pose() {
      return new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    protected void reset() {
      x = 0;
      y = 0;
      z = 0;

      vx = 0;
      vy = 0;
      vz = 0;

      ax = 0;
      ay = 0;
      az = 0;

      roll = 0;
      pitch = 0;
      yaw = 0;

      omega = 0;
      alpha = 0;

      frames = 0;
    }

    protected static double norm(double[] vector) {
      double x = vector[X];
      double y = vector[Y];
      double z = vector[Z];

      return Math.sqrt(x * x + y * y + z * z);
    }

    protected static double[] fromTranslation(Translation3d translation) {
      return new double[] {translation.getX(), translation.getY(), translation.getZ()};
    }
  }
}
