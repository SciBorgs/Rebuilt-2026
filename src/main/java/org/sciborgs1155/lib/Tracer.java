package org.sciborgs1155.lib;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

/**
 * A utility class for tracing code execution time. Will put info to NetworkTables under the
 * "Tracer" table.
 *
 * <p>Example inside {@code Robot.java}
 *
 * <pre><code>
 *
 * public void robotPeriodic() {
 *   Tracer.trace("CommandScheduler", CommandScheduler.getInstance()::run); // CommandScheduler is already traced
 *   Tracer.trace("MyVendorDep", MyVendorDep::updateAll);
 * }
 * </code></pre>
 *
 * <p>Example inside a {@code Drive Subsystem}
 *
 * <pre><code>
 * // Subsystem periodics are automaticall traced
 * public void periodic() {
 *   for (var module : modules) {
 *     Tracer.traceFunc("Module" + module.getName(), module::update);
 *   }
 *   Tracer.traceFunc("Gyro", gyro::update);
 * }
 * </code></pre>
 *
 * @see https://github.com/wpilibsuite/allwpilib/pull/7099
 */
@SuppressWarnings("PMD.UseConcurrentHashMap") // Not sure if concurrent access
public final class Tracer {

  // Prevents instantiation
  private Tracer() {}

  private static final class TraceStartData {
    private double mStartTime;
    private double mStartGCTotalTime;

    private void set(double startTime, double startGCTotalTime) {
      this.mStartTime = startTime;
      this.mStartGCTotalTime = startGCTotalTime;
    }
  }

  /** All the tracers persistent state in a single object to be stored in a {@link ThreadLocal}. */
  @SuppressWarnings("PMD.RedundantFieldInitializer")
  private static final class TracerState {
    private final NetworkTable mRootTable;

    /**
     * The stack of traces, every startTrace will add to this stack and every endTrace will remove
     * from this stack.
     */
    private final List<String> mTraceStack = new ArrayList<>();

    /**
     * ideally we only need `traceStack` but in the interest of memory optimization and string
     * concatenation speed we store the history of the stack to reuse the stack names.
     */
    private final List<String> mTraceStackHistory = new ArrayList<>();

    /** The time of each trace, the key is the trace name, modified every endTrace. */
    private final Map<String, Double> mTraceTimes = new HashMap<>();

    /**
     * The start time of each trace and the gc time at the start of the trace, the key is the trace
     * name, modified every startTrace and endTrace.
     */
    private final Map<String, TraceStartData> mTraceStartTimes = new HashMap<>();

    /** the publishers for each trace, the key is the trace name, modified every endCycle. */
    private final Map<String, DoublePublisher> mPublishers = new HashMap<>();

    /*
     * If the cycle is poisoned, it will warn the user
     * and not publish any data.
     */
    boolean mCyclePoisoned = false;

    /** If the tracer is disabled, it will not publish any data or do any string manipulation. */
    boolean mDisabled = false;

    /**
     * If the tracer should be disabled next cycle and every cycle after that until this flag is set
     * to false. Disabling is done this way to prevent disabling/enabling.
     */
    boolean mDisableNextCycle = false;

    /**
     * Stack size is used to keep track of stack size even when disabled, calling `EndCycle` is
     * important when disabled or not to update the disabled state in a safe manner.
     */
    int mStackSize = 0;

    // the garbage collector beans
    private final List<GarbageCollectorMXBean> mGcs =
        new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());
    private final DoublePublisher mGcTimeEntry;
    private double mGcTimeThisCycle = 0.0;

    private TracerState(String name, boolean threadLocalConstruction) {
      if (SINGLE_THREADED_MODE.get() && threadLocalConstruction) {
        DriverStation.reportError(
            "[Tracer] Tracer is in single threaded mode, cannot start traces on multiple threads",
            true);
        this.mDisabled = true;
      }
      ANY_TRACES_STARTED.set(true);
      if (name == null) {
        this.mRootTable = NetworkTableInstance.getDefault().getTable("Tracer");
      } else {
        this.mRootTable = NetworkTableInstance.getDefault().getTable("Tracer").getSubTable(name);
      }
      this.mGcTimeEntry = mRootTable.getDoubleTopic("GCTime").publish();
    }

    private String appendTraceStack(String trace) {
      mStackSize++;
      if (mDisabled) {
        return "";
      }
      mTraceStack.add(trace);
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < mTraceStack.size(); i++) {
        sb.append(mTraceStack.get(i));
        if (i < mTraceStack.size() - 1) {
          sb.append('/');
        }
      }
      String str = sb.toString();
      mTraceStackHistory.add(str);
      return str;
    }

    private String popTraceStack() {
      mStackSize = Math.max(0, mStackSize - 1);
      if (mDisabled) {
        return "";
      }
      if (mTraceStack.isEmpty() || mTraceStackHistory.isEmpty() || mCyclePoisoned) {
        mCyclePoisoned = true;
        return "";
      }
      mTraceStack.remove(mTraceStack.size() - 1);
      return mTraceStackHistory.remove(mTraceStackHistory.size() - 1);
    }

    private double totalGCTime() {
      double gcTime = 0;
      for (GarbageCollectorMXBean gc : mGcs) {
        gcTime += gc.getCollectionTime() / 1000.0;
      }
      return gcTime;
    }

    private void endCycle() {
      if (mDisabled != mDisableNextCycle || mCyclePoisoned) {
        // Gives publishers empty times,
        // reporting no data is better than bad data
        for (var publisher : mPublishers.entrySet()) {
          publisher.getValue().set(0.0);
        }
        return;
      } else if (!mDisabled) {
        // update times for all already existing publishers
        for (var publisher : mPublishers.entrySet()) {
          Double time = mTraceTimes.remove(publisher.getKey());
          if (time == null) {
            time = 0.0;
          }
          publisher.getValue().set(time);
        }
        // create publishers for all new entries
        for (var traceTime : mTraceTimes.entrySet()) {
          DoublePublisher publisher = mRootTable.getDoubleTopic(traceTime.getKey()).publish();
          publisher.set(traceTime.getValue());
          mPublishers.put(traceTime.getKey(), publisher);
        }
        // log gc time
        if (!mGcs.isEmpty()) {
          mGcTimeEntry.set(mGcTimeThisCycle);
        }
        mGcTimeThisCycle = 0.0;
      }

      // clean up state
      mTraceTimes.clear();
      mTraceStackHistory.clear();

      mDisabled = mDisableNextCycle;
    }
  }

  private static final AtomicBoolean SINGLE_THREADED_MODE = new AtomicBoolean(false);
  private static final AtomicBoolean ANY_TRACES_STARTED = new AtomicBoolean(false);
  private static final ThreadLocal<TracerState> THREAD_LOCAL_STATE =
      ThreadLocal.withInitial(() -> new TracerState(Thread.currentThread().getName(), true));

  private static void startTraceInner(final String name, final TracerState state) {
    String stack = state.appendTraceStack(name);
    if (state.mDisabled) {
      return;
    }
    TraceStartData data = state.mTraceStartTimes.computeIfAbsent(stack, k -> new TraceStartData());
    data.set(Timer.getFPGATimestamp(), state.totalGCTime());
  }

  private static void endTraceInner(final TracerState state) {
    String stack = state.popTraceStack();
    if (!state.mDisabled) {
      if (stack.isEmpty()) {
        DriverStation.reportError(
            "[Tracer] Stack is empty,"
                + "this means that there are more endTrace calls than startTrace calls",
            true);
        return;
      }
      var startData = state.mTraceStartTimes.get(stack);
      double gcTimeSinceStart = state.totalGCTime() - startData.mStartGCTotalTime;
      state.mGcTimeThisCycle += gcTimeSinceStart;
      state.mTraceTimes.put(
          stack, Timer.getFPGATimestamp() - startData.mStartTime - gcTimeSinceStart);
    }
    if (state.mTraceStack.isEmpty()) {
      state.endCycle();
    }
  }

  /**
   * Starts a trace, should be called at the beginning of a function that's not being called by user
   * code.
   *
   * @param name the name of the trace, should be unique to the function.
   */
  public static void startTrace(String name) {
    startTraceInner(name, THREAD_LOCAL_STATE.get());
  }

  /**
   * Ends a trace, should only be called at the end of a function that's not being called by user
   * code. If a {@link Tracer#startTrace(String)} is not paired with an endTrace() call there could
   * be dropped or incorrect data.
   */
  public static void endTrace() {
    endTraceInner(THREAD_LOCAL_STATE.get());
  }

  /**
   * Disables garbage collection logging for the current thread. This can help performance in some
   * cases.
   *
   * <p>This counts as starting a tracer on the current thread, this is important to consider with
   * {@link #enableSingleThreadedMode()} and should never be called before if you are using single
   * threaded mode.
   */
  public static void disableGcLoggingForCurrentThread() {
    TracerState state = THREAD_LOCAL_STATE.get();
    state.mGcTimeEntry.close();
    state.mGcs.clear();
  }

  /**
   * Enables single threaded mode for the Tracer. This will cause traces on different threads to
   * throw an exception. This will shorten the path of traced data in NetworkTables by not including
   * the thread name.
   *
   * <p>This function should be called before any traces are started.
   */
  public static void enableSingleThreadedMode() {
    if (ANY_TRACES_STARTED.get()) {
      DriverStation.reportError(
          "[Tracer] Cannot enable single-threaded mode after traces have been started", true);
    } else {
      THREAD_LOCAL_STATE.set(new TracerState(null, false));
      SINGLE_THREADED_MODE.set(true);
    }
  }

  /**
   * Disables any tracing for the current thread. This will cause all {@link #startTrace(String)},
   * {@link #endTrace()} and {@link #trace(String, Runnable)} to do nothing.
   *
   * <p>Being disabled prevents the {@link Tracer} from publishing any values to NetworkTables. This
   * will cause all values to appear as if they're frozen at the value they were at when this
   * function was called.
   */
  public static void disableTracingForCurrentThread() {
    final TracerState state = THREAD_LOCAL_STATE.get();
    state.mDisableNextCycle = true;
  }

  /**
   * Enables any tracing for the current thread. This will cause all {@link #startTrace(String)},
   * {@link #endTrace()} and {@link #trace(String, Runnable)} to work as normal.
   */
  public static void enableTracingForCurrentThread() {
    final TracerState state = THREAD_LOCAL_STATE.get();
    state.mDisableNextCycle = false;
    if (state.mStackSize == 0) {
      state.mDisabled = false;
    }
  }

  /**
   * Traces a function, should be used in place of {@link #startTrace(String)} and {@link
   * #endTrace()} for functions called by user code like {@code CommandScheduler.run()} and other
   * expensive functions.
   *
   * @param name the name of the trace, should be unique to the function.
   * @param runnable the function to trace.
   */
  public static void trace(String name, Runnable runnable) {
    final TracerState state = THREAD_LOCAL_STATE.get();
    startTraceInner(name, state);
    runnable.run();
    endTraceInner(state);
  }

  /**
   * Traces a function, should be used in place of {@link #startTrace(String)} and {@link
   * #endTrace()} for functions called by user code.
   *
   * @param <R> the return type of the function.
   * @param name the name of the trace, should be unique to the function.
   * @param supplier the function to trace.
   * @return the return value of the function.
   */
  public static <R> R trace(String name, Supplier<R> supplier) {
    final TracerState state = THREAD_LOCAL_STATE.get();
    startTraceInner(name, state);
    try {
      return supplier.get();
    } finally {
      endTraceInner(state);
    }
  }

  /** This function is only to be used in tests and is package private to prevent misuse. */
  static void resetForTest() {
    THREAD_LOCAL_STATE.remove();
    SINGLE_THREADED_MODE.set(false);
    ANY_TRACES_STARTED.set(false);
  }
}
