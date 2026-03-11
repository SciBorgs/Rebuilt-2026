package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.X;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Y;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.Z;

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

@SuppressWarnings("PMD.OneDeclarationPerLine")
public abstract class ProjectileVisualizer {

  private static final Pose3d[] EMPTY_POSES = new Pose3d[0];

  /** tolerance used to detect input changes */
  protected static final double EPS = 1e-6;

  private double airTime;
  private int scores, misses;
  private boolean willScore, willMiss;

  private boolean launchEnabled = true;
  private boolean trajectoryEnabled = true;

  private boolean weightEnabled = true;
  private boolean dragEnabled = true;
  private boolean torqueEnabled = false;
  private boolean liftEnabled = false;

  private double launchResolution = 100;
  private double trajectoryResolution = 100;
  private double cooldown = 0.05;
  private double maxAirTime = 3.0;

  private int maxLaunchFrames;
  private int maxTrajectoryFrames;

  private double launchDt;
  private double trajectoryDt;

  private Pose3d initial;
  private Pose3d ending;

  private volatile Pose3d[] trajectory = EMPTY_POSES;

  private final Supplier<double[]> initialTranslation;
  private final Supplier<double[]> initialVelocity;
  private final Supplier<double[]> initialRotation;
  private final DoubleSupplier initialRotationalVelocity;

  private final List<Projectile> projectiles = new ArrayList<>();

  /** object pool for projectiles */
  private final ArrayDeque<Projectile> projectilePool = new ArrayDeque<>();

  /** reusable trajectory buffer */
  private final List<Pose3d> trajectoryBuffer = new ArrayList<>(512);

  /** cached launch state for change detection */
  private final double[] lastInitialTranslation = new double[3];

  private final double[] lastInitialVelocity = new double[3];
  private final double[] lastInitialRotation = new double[3];
  private double lastInitialRotationalVelocity;

  private boolean launchStateInitialized = false;

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

  public ProjectileVisualizer(
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

  // ------------------------------------------------
  // LAUNCH STATE CHANGE DETECTION
  // ------------------------------------------------

  private static boolean diff(double a, double b) {
    return Math.abs(a - b) > EPS;
  }

  private void checkLaunchState() {

    double[] translation = initialTranslation.get();
    double[] velocity = initialVelocity.get();
    double[] rotation = initialRotation.get();
    double rotationalVelocity = initialRotationalVelocity.getAsDouble();

    boolean changed = false;

    if (!launchStateInitialized) {

      changed = true;
      launchStateInitialized = true;

    } else {

      if (diff(translation[X], lastInitialTranslation[X])
          || diff(translation[Y], lastInitialTranslation[Y])
          || diff(translation[Z], lastInitialTranslation[Z])) changed = true;

      if (diff(velocity[X], lastInitialVelocity[X])
          || diff(velocity[Y], lastInitialVelocity[Y])
          || diff(velocity[Z], lastInitialVelocity[Z])) changed = true;

      if (diff(rotation[X], lastInitialRotation[X])
          || diff(rotation[Y], lastInitialRotation[Y])
          || diff(rotation[Z], lastInitialRotation[Z])) changed = true;

      if (diff(rotationalVelocity, lastInitialRotationalVelocity)) changed = true;
    }

    if (changed) {

      System.arraycopy(translation, 0, lastInitialTranslation, 0, 3);
      System.arraycopy(velocity, 0, lastInitialVelocity, 0, 3);
      System.arraycopy(rotation, 0, lastInitialRotation, 0, 3);

      lastInitialRotationalVelocity = rotationalVelocity;

      trajectoryDirty = true;
    }
  }

  // ------------------------------------------------
  // OBJECT POOL
  // ------------------------------------------------

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

  // ------------------------------------------------
  // SIMULATION CONTROL
  // ------------------------------------------------

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

  public void endSimulation() {
    running.set(false);
  }

  // ------------------------------------------------
  // CONFIGURATION
  // ------------------------------------------------

  public ProjectileVisualizer configPhysics(
      boolean weight, boolean drag, boolean torque, boolean lift) {

    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    trajectoryDirty = true;

    return this;
  }

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

  public ProjectileVisualizer config(boolean launch, boolean trajectory) {

    launchEnabled = launch;
    trajectoryEnabled = trajectory;

    return this;
  }

  // ------------------------------------------------
  // PROJECTILE LAUNCH
  // ------------------------------------------------

  public void launchProjectile() {

    if (!running.get()) return;

    Projectile projectile = obtainProjectile();

    projectile.launch(
        initialTranslation.get(),
        initialVelocity.get(),
        initialRotation.get(),
        initialRotationalVelocity.getAsDouble());

    synchronized (projectiles) {
      projectiles.add(projectile);
    }

    initial = projectile.pose();
  }

  public Command launchProjectiles() {

    return Commands.repeatingSequence(
        Commands.runOnce(this::launchProjectile).andThen(Commands.waitSeconds(cooldown)));
  }

  // ------------------------------------------------
  // SIMULATION LOOPS
  // ------------------------------------------------

  private void updateLaunchSimulation() {

    if (!launchEnabled) return;

    synchronized (projectiles) {
      for (int i = projectiles.size() - 1; i >= 0; i--) {

        Projectile projectile = projectiles.get(i);

        if (projectile.willMiss()) {
          misses++;
          ending = projectile.pose();
          projectiles.remove(i);
          recycleProjectile(projectile);
          continue;
        }

        if (projectile.willScore()) {
          scores++;
          ending = projectile.pose();
          projectiles.remove(i);
          recycleProjectile(projectile);
          continue;
        }

        if (projectile.frames >= maxLaunchFrames) {
          ending = projectile.pose();
          projectiles.remove(i);
          recycleProjectile(projectile);
          continue;
        }

        projectile.periodic();
      }
    }
  }

  private void updateTrajectorySimulation() {

    checkLaunchState();

    if (!trajectoryEnabled || !trajectoryDirty) return;

    trajectory = generateTrajectory();

    trajectoryDirty = false;
  }

  // ------------------------------------------------
  // TRAJECTORY GENERATION
  // ------------------------------------------------

  public Pose3d[] generateTrajectory() {

    trajectoryBuffer.clear();

    Projectile projectile = obtainProjectile();

    projectile.config(trajectoryResolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);

    projectile.launch(
        initialTranslation.get(),
        initialVelocity.get(),
        initialRotation.get(),
        initialRotationalVelocity.getAsDouble());

    initial = projectile.pose();

    int frames = 0;

    while (!projectile.willMiss() && !projectile.willScore() && frames <= maxTrajectoryFrames) {

      trajectoryBuffer.add(projectile.pose());

      projectile.periodic();

      frames++;
    }

    ending = projectile.pose();

    willMiss = projectile.willMiss();
    willScore = projectile.willScore();

    airTime = frames / trajectoryResolution;

    recycleProjectile(projectile);

    return trajectoryBuffer.toArray(new Pose3d[0]);
  }

  // ------------------------------------------------
  // DATA ACCESS
  // ------------------------------------------------

  public Pose3d[] poses() {

    synchronized (projectiles) {
      Pose3d[] poses = new Pose3d[projectiles.size()];

      for (int index = 0; index < poses.length; index++)
        poses[index] = projectiles.get(index).pose();

      return poses;
    }
  }

  public Pose3d[] trajectory() {
    return trajectory.clone();
  }

  public Pose3d initial() {
    return initial;
  }

  public Pose3d ending() {
    return ending;
  }

  public boolean willScore() {
    return willScore;
  }

  public boolean willMiss() {
    return willMiss;
  }

  public double airTime() {
    return airTime;
  }

  public int scores() {
    return scores;
  }

  public int misses() {
    return misses;
  }

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

  // =========================================================
  // PROJECTILE PHYSICS
  // =========================================================

  protected abstract static class Projectile {
    protected static final double GRAVITY = -9.80665;

    protected static final int X = 0, Y = 1, Z = 2;
    protected static final int SPEED = 0, PITCH = 1, YAW = 2;

    protected int frames;

    protected double resolution;
    protected double dt;

    protected boolean weightEnabled;
    protected boolean dragEnabled;
    protected boolean torqueEnabled;
    protected boolean liftEnabled;

    // =============================
    // PHYSICS STATE (NO ARRAYS)
    // =============================

    protected double x, y, z;
    protected double vx, vy, vz;
    protected double ax, ay, az;

    // Euler rotation
    protected double roll;
    protected double pitch;
    protected double yaw;

    protected double omega;
    protected double alpha;

    // =============================
    // PHYSICS MODEL HOOKS
    // =============================

    protected double weight() {
      return GRAVITY;
    }

    protected abstract double[] drag();

    protected double torque() {
      return 0;
    }

    protected double[] lift() {
      return new double[] {0, 0, 0};
    }

    protected abstract boolean willScore();

    protected abstract boolean willMiss();

    // =============================
    // CONFIGURATION
    // =============================

    protected Projectile config(
        double fps, boolean weight, boolean drag, boolean torque, boolean lift) {

      resolution = fps;
      dt = 1.0 / fps;

      weightEnabled = weight;
      dragEnabled = drag;
      torqueEnabled = torque;
      liftEnabled = lift;

      return this;
    }

    // =============================
    // LAUNCH
    // =============================

    protected void launch(
        double[] launchTranslation,
        double[] launchVelocity,
        double[] launchRotation,
        double launchRotationalVelocity) {

      x = launchTranslation[X];
      y = launchTranslation[Y];
      z = launchTranslation[Z];

      vx = launchVelocity[X];
      vy = launchVelocity[Y];
      vz = launchVelocity[Z];

      ax = ay = az = 0;

      roll = launchRotation[0];
      pitch = launchRotation[1];
      yaw = launchRotation[2];

      omega = launchRotationalVelocity;
      alpha = 0;

      frames = 0;
    }

    // =============================
    // PHYSICS STEP
    // =============================

    protected void periodic() {

      // integrate position
      x += vx * dt;
      y += vy * dt;
      z += vz * dt;

      // integrate velocity
      vx += ax * dt;
      vy += ay * dt;
      vz += az * dt;

      // reset acceleration
      ax = ay = az = 0;

      // weight
      if (weightEnabled) az += GRAVITY;

      // drag
      if (dragEnabled) {
        double[] d = drag();
        ax += d[X];
        ay += d[Y];
        az += d[Z];
      }

      // lift
      if (liftEnabled) {
        double[] l = lift();
        ax += l[X];
        ay += l[Y];
        az += l[Z];
      }

      // integrate rotation (pitch)
      pitch += omega * dt;

      omega += alpha * dt;
      alpha = torqueEnabled ? torque() : 0;

      frames++;
    }

    // =============================
    // POSE OUTPUT
    // =============================

    protected Pose3d pose() {

      return new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    // =============================
    // RESET FOR OBJECT POOL
    // =============================

    protected void reset() {

      x = y = z = 0;

      vx = vy = vz = 0;

      ax = ay = az = 0;

      roll = pitch = yaw = 0;

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
