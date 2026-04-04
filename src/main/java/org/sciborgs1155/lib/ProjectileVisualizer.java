package org.sciborgs1155.lib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

@SuppressWarnings("PMD")
public abstract class ProjectileVisualizer {
  /** The first, second, and third values of a 3D vector correspond to X, Y, and Z. */
  public static final int X = 0, Y = 1, Z = 2;

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

  // INITIAL STATE SUPPLIERS
  private final DoubleSupplier initialTranslationX, initialTranslationY, initialTranslationZ;
  private final DoubleSupplier initialVelocityX, initialVelocityY, initialVelocityZ;
  private final DoubleSupplier initialRotationX, initialRotationY, initialRotationZ;
  private final DoubleSupplier initialRotationalVelocity;

  private final List<Projectile> projectiles = new ArrayList<>();

  /** Object pool for projectiles. */
  private final Deque<Projectile> projectilePool = new ArrayDeque<>();

  /** Reusable trajectory buffer. */
  private final List<Pose3d> trajectoryBuffer = new ArrayList<>(512);

  // CACHED LAUNCH STATE
  private double lastTranslationX, lastTranslationY, lastTranslationZ;
  private double lastVelocityX, lastVelocityY, lastVelocityZ;
  private double lastRotationX, lastRotationY, lastRotationZ;
  private double lastRotationalVelocity;

  private boolean launchStateInitialized;
  private volatile boolean trajectoryDirty = true;

  private final ScheduledExecutorService executor =
      Executors.newScheduledThreadPool(
          2, runnable -> new Thread(runnable, "Projectile Visualizer"));

  private final AtomicBoolean running = new AtomicBoolean(false);

  protected abstract Projectile createProjectile();

  protected ProjectileVisualizer(
      DoubleSupplier initialTranslationX,
      DoubleSupplier initialTranslationY,
      DoubleSupplier initialTranslationZ,
      DoubleSupplier initialVelocityX,
      DoubleSupplier initialVelocityY,
      DoubleSupplier initialVelocityZ,
      DoubleSupplier initialRotationX,
      DoubleSupplier initialRotationY,
      DoubleSupplier initialRotationZ,
      DoubleSupplier initialRotationalVelocity) {
    this.initialTranslationX = initialTranslationX;
    this.initialTranslationY = initialTranslationY;
    this.initialTranslationZ = initialTranslationZ;
    this.initialVelocityX = initialVelocityX;
    this.initialVelocityY = initialVelocityY;
    this.initialVelocityZ = initialVelocityZ;
    this.initialRotationX = initialRotationX;
    this.initialRotationY = initialRotationY;
    this.initialRotationZ = initialRotationZ;
    this.initialRotationalVelocity = initialRotationalVelocity;

    recomputeFrameLimits();
  }

  private void recomputeFrameLimits() {
    maxLaunchFrames = (int) (maxAirTime * launchResolution);
    maxTrajectoryFrames = (int) (maxAirTime * trajectoryResolution);

    launchDt = 1.0 / launchResolution;
    trajectoryDt = 1.0 / trajectoryResolution;
  }

  private static boolean diff(double a, double b) {
    return Math.abs(a - b) > 1e-6;
  }

  private void checkLaunchState() {
    double tx = initialTranslationX.getAsDouble();
    double ty = initialTranslationY.getAsDouble();
    double tz = initialTranslationZ.getAsDouble();
    double vx = initialVelocityX.getAsDouble();
    double vy = initialVelocityY.getAsDouble();
    double vz = initialVelocityZ.getAsDouble();
    double rx = initialRotationX.getAsDouble();
    double ry = initialRotationY.getAsDouble();
    double rz = initialRotationZ.getAsDouble();
    double rv = initialRotationalVelocity.getAsDouble();

    boolean changed =
        !launchStateInitialized
            || diff(tx, lastTranslationX)
            || diff(ty, lastTranslationY)
            || diff(tz, lastTranslationZ)
            || diff(vx, lastVelocityX)
            || diff(vy, lastVelocityY)
            || diff(vz, lastVelocityZ)
            || diff(rx, lastRotationX)
            || diff(ry, lastRotationY)
            || diff(rz, lastRotationZ)
            || diff(rv, lastRotationalVelocity);

    if (changed) {
      lastTranslationX = tx;
      lastTranslationY = ty;
      lastTranslationZ = tz;
      lastVelocityX = vx;
      lastVelocityY = vy;
      lastVelocityZ = vz;
      lastRotationX = rx;
      lastRotationY = ry;
      lastRotationZ = rz;
      lastRotationalVelocity = rv;

      launchStateInitialized = true;
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
        this::updateLaunchSimulation, 0, (long) (launchDt * 1_000_000), TimeUnit.MICROSECONDS);

    executor.scheduleAtFixedRate(
        this::updateTrajectorySimulation,
        0,
        (long) (trajectoryDt * 1_000_000),
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
    if (!running.get() || !launchEnabled) return;
    Projectile projectile = obtainProjectile();

    projectile.initialize(
        initialTranslationX.getAsDouble(),
        initialTranslationY.getAsDouble(),
        initialTranslationZ.getAsDouble(),
        initialVelocityX.getAsDouble(),
        initialVelocityY.getAsDouble(),
        initialVelocityZ.getAsDouble(),
        initialRotationX.getAsDouble(),
        initialRotationY.getAsDouble(),
        initialRotationZ.getAsDouble(),
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
        lastTranslationX,
        lastTranslationY,
        lastTranslationZ,
        lastVelocityX,
        lastVelocityY,
        lastVelocityZ,
        lastRotationX,
        lastRotationY,
        lastRotationZ,
        lastRotationalVelocity);

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

  public static class Projectile {
    public static final double GRAVITY = -9.80665;
    public static final double AIR_DENSITY = 1.225;

    protected int frames;
    protected double resolution, delta, halfDelta, dt6;
    protected boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;
    protected double x, y, z;
    protected double vx, vy, vz;
    protected double ax, ay, az;
    protected double roll, pitch, yaw;
    protected double omega;
    protected double alpha;

    // Pre-allocated RK4 working buffers — never allocate in step()
    private final double[] s0 = new double[8];
    private final double[] k1 = new double[8];
    private final double[] k2 = new double[8];
    private final double[] k3 = new double[8];
    private final double[] k4 = new double[8];
    private final double[] tmp = new double[8];

    // Pre-allocated force output buffers
    private final double[] dragOut = new double[3];
    private final double[] liftOut = new double[3];

    // Reusable state carrier — never escapes derivatives()
    private final MutableState stateCarrier = new MutableState();

    // Defers the 5th derivatives() call until ax/ay/az/alpha are actually read
    private boolean accelDirty = true;

    // Strategy functions with no-op defaults
    private ForceFunction dragFunction =
        (s, out) -> {
          out[0] = 0;
          out[1] = 0;
          out[2] = 0;
        };
    private ForceFunction liftFunction =
        (s, out) -> {
          out[0] = 0;
          out[1] = 0;
          out[2] = 0;
        };
    private ToDoubleFunction<MutableState> torqueFunction = s -> 0.0;
    private Predicate<MutableState> scoreFunction = s -> false;
    private Predicate<MutableState> missFunction = s -> false;

    /**
     * Mutable snapshot of projectile state passed to force functions. Reused on every step to avoid
     * per-step allocation.
     */
    public final class MutableState {
      public double x, y, z, vx, vy, vz, pitch, omega, roll, yaw;

      /** Loads from an RK4 intermediate state array. Roll and yaw are unchanged during a step. */
      void load(double[] s) {
        x = s[0];
        y = s[1];
        z = s[2];
        vx = s[3];
        vy = s[4];
        vz = s[5];
        pitch = s[6];
        omega = s[7];
        roll = Projectile.this.roll;
        yaw = Projectile.this.yaw;
      }

      /** Loads directly from the projectile's current field state. */
      void loadCurrent() {
        x = Projectile.this.x;
        y = Projectile.this.y;
        z = Projectile.this.z;
        vx = Projectile.this.vx;
        vy = Projectile.this.vy;
        vz = Projectile.this.vz;
        pitch = Projectile.this.pitch;
        omega = Projectile.this.omega;
        roll = Projectile.this.roll;
        yaw = Projectile.this.yaw;
      }
    }

    /**
     * Computes a force vector given the current state. Writes {ax, ay, az} into the provided output
     * buffer rather than allocating a new array.
     */
    @FunctionalInterface
    public interface ForceFunction {
      void apply(MutableState state, double[] out);
    }

    public Projectile config(
        double fps, boolean weight, boolean drag, boolean torque, boolean lift) {
      resolution = fps;
      delta = 1.0 / fps;
      halfDelta = 0.5 * delta;
      dt6 = delta / 6.0;
      weightEnabled = weight;
      dragEnabled = drag;
      torqueEnabled = torque;
      liftEnabled = lift;

      return this;
    }

    public Projectile withDrag(ForceFunction f) {
      dragFunction = f;
      return this;
    }

    public Projectile withLift(ForceFunction f) {
      liftFunction = f;
      return this;
    }

    public Projectile withTorque(ToDoubleFunction<MutableState> f) {
      torqueFunction = f;
      return this;
    }

    public Projectile withScore(Predicate<MutableState> f) {
      scoreFunction = f;
      return this;
    }

    public Projectile withMiss(Predicate<MutableState> f) {
      missFunction = f;
      return this;
    }

    public double x() {
      return x;
    }

    public double y() {
      return y;
    }

    public double z() {
      return z;
    }

    /** Recomputes acceleration lazily — only if the state has changed since last read. */
    public double ax() {
      if (accelDirty) refreshAccel();
      return ax;
    }

    public double ay() {
      if (accelDirty) refreshAccel();
      return ay;
    }

    public double az() {
      if (accelDirty) refreshAccel();
      return az;
    }

    public double alpha() {
      if (accelDirty) refreshAccel();
      return alpha;
    }

    public boolean willScore() {
      stateCarrier.loadCurrent();
      return scoreFunction.test(stateCarrier);
    }

    public boolean willMiss() {
      stateCarrier.loadCurrent();
      return missFunction.test(stateCarrier);
    }

    /** Replaces NaN with 0.0 so invalid supplier values are handled gracefully. */
    protected static double validated(double value) {
      return value == value ? value : 0.0;
    }

    protected void initialize(
        double tx,
        double ty,
        double tz,
        double ivx,
        double ivy,
        double ivz,
        double rx,
        double ry,
        double rz,
        double rotationalVelocity) {
      x = validated(tx);
      y = validated(ty);
      z = validated(tz);
      vx = validated(ivx);
      vy = validated(ivy);
      vz = validated(ivz);
      ax = 0;
      ay = 0;
      az = 0;
      roll = validated(rx);
      pitch = validated(ry);
      yaw = validated(rz);
      omega = validated(rotationalVelocity);
      alpha = 0;
      frames = 0;
      accelDirty = true;
    }

    public void initialize(
        double[] translation,
        double[] initialVelocity,
        double[] initialRotation,
        double rotationalVelocity) {
      initialize(
          translation[X],
          translation[Y],
          translation[Z],
          initialVelocity[X],
          initialVelocity[Y],
          initialVelocity[Z],
          initialRotation[X],
          initialRotation[Y],
          initialRotation[Z],
          rotationalVelocity);
    }

    /**
     * Advances the simulation by one timestep using RK4.
     *
     * <p>State vector: [x, y, z, vx, vy, vz, pitch, omega] Derivative vector: [vx, vy, vz, ax, ay,
     * az, omega, alpha]
     */
    public void step() {
      s0[0] = x;
      s0[1] = y;
      s0[2] = z;
      s0[3] = vx;
      s0[4] = vy;
      s0[5] = vz;
      s0[6] = pitch;
      s0[7] = omega;

      derivatives(s0, k1);
      advanceState(s0, k1, halfDelta, tmp);
      derivatives(tmp, k2);
      advanceState(s0, k2, halfDelta, tmp);
      derivatives(tmp, k3);
      advanceState(s0, k3, delta, tmp);
      derivatives(tmp, k4);

      x = s0[0] + dt6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      y = s0[1] + dt6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      z = s0[2] + dt6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
      vx = s0[3] + dt6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
      vy = s0[4] + dt6 * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);
      vz = s0[5] + dt6 * (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]);
      pitch = s0[6] + dt6 * (k1[6] + 2 * k2[6] + 2 * k3[6] + k4[6]);
      omega = s0[7] + dt6 * (k1[7] + 2 * k2[7] + 2 * k3[7] + k4[7]);

      accelDirty = true;
      frames++;
    }

    /** Evaluates the derivative vector for a given state, writing the result into {@code out}. */
    private void derivatives(double[] s, double[] out) {
      stateCarrier.load(s);

      double dax = 0, day = 0, daz = 0;

      if (weightEnabled) daz += GRAVITY;

      if (dragEnabled) {
        dragFunction.apply(stateCarrier, dragOut);
        dax += dragOut[0];
        day += dragOut[1];
        daz += dragOut[2];
      }

      if (liftEnabled) {
        liftFunction.apply(stateCarrier, liftOut);
        dax += liftOut[0];
        day += liftOut[1];
        daz += liftOut[2];
      }

      double dalpha = torqueEnabled ? torqueFunction.applyAsDouble(stateCarrier) : 0.0;

      out[0] = s[3];
      out[1] = s[4];
      out[2] = s[5]; // dx/dt  = velocity
      out[3] = dax;
      out[4] = day;
      out[5] = daz; // dv/dt  = acceleration
      out[6] = s[7];
      out[7] = dalpha; // dpitch/dt = omega, domega/dt = alpha
    }

    /**
     * Recomputes ax/ay/az/alpha from the current state. Called at most once per step, and only if
     * those values are actually read.
     */
    private void refreshAccel() {
      s0[0] = x;
      s0[1] = y;
      s0[2] = z;
      s0[3] = vx;
      s0[4] = vy;
      s0[5] = vz;
      s0[6] = pitch;
      s0[7] = omega;
      derivatives(s0, k1);
      ax = k1[3];
      ay = k1[4];
      az = k1[5];
      alpha = k1[7];
      accelDirty = false;
    }

    /** Computes {@code out = s + scale * k} in-place. */
    private static void advanceState(double[] s, double[] k, double scale, double[] out) {
      for (int i = 0; i < 8; i++) out[i] = s[i] + scale * k[i];
    }

    public Pose3d pose() {
      return new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    public void reset() {
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
      accelDirty = true;
    }
  }

  /** Caches a recomputed double[3] and exposes per-component DoubleSuppliers. */
  protected static final class CachedVector {
    private final Supplier<double[]> source;
    private final double[] cache = new double[3];
    private boolean dirty = true;

    public CachedVector(Supplier<double[]> source) {
      this.source = source;
    }

    private void refresh() {
      if (!dirty) return;
      double[] result = source.get();
      cache[0] = result[0];
      cache[1] = result[1];
      cache[2] = result[2];
      dirty = false;
    }

    public void invalidate() {
      dirty = true;
    }

    public DoubleSupplier x() {
      return () -> {
        refresh();
        return cache[X];
      };
    }

    public DoubleSupplier y() {
      return () -> {
        refresh();
        return cache[Y];
      };
    }

    public DoubleSupplier z() {
      return () -> {
        refresh();
        return cache[Z];
      };
    }
  }

  public static double norm(double[] vector) {
    double x = vector[X];
    double y = vector[Y];
    double z = vector[Z];

    return Math.sqrt(x * x + y * y + z * z);
  }

  public static double[] fromTranslation(Translation3d translation) {
    return new double[] {translation.getX(), translation.getY(), translation.getZ()};
  }
}
