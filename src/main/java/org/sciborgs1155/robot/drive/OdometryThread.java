package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.DRIVE_CANIVORE;
import static org.sciborgs1155.robot.Constants.ODOMETRY_PERIOD;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * A class for faster Talon Odometry using a faster thread.
 *
 * <p>Inspired by 6328's PhoenixOdometryThread.
 */
public class OdometryThread extends Thread {
  private BaseStatusSignal[] talonSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> talonQueues = new ArrayList<>();
  private final List<DoubleSupplier> otherSignals = new ArrayList<>();
  private final List<Queue<Double>> otherQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static boolean isCANFD = DRIVE_CANIVORE.isNetworkFD();
  private static OdometryThread instance;

  public static synchronized OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  @Override
  public synchronized void start() {
    if (!timestampQueues.isEmpty()) {
      super.start();
    }
  }

  /**
   * Registers a Talon status signal for odometry updates.
   *
   * @param signal The status signal to register.
   * @return A queue containing the signal values.
   */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.LOCK.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[talonSignals.length + 1];
      System.arraycopy(talonSignals, 0, newSignals, 0, talonSignals.length);
      newSignals[talonSignals.length] = signal;
      talonSignals = newSignals;
      talonQueues.add(queue);
    } finally {
      Drive.LOCK.unlock();
    }
    return queue;
  }

  /**
   * Registers a generic double supplier for odometry updates.
   *
   * @param signal The double supplier to register.
   * @return A queue containing the signal values.
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.LOCK.lock();
    try {
      otherSignals.add(signal);
      otherQueues.add(queue);
    } finally {
      Drive.LOCK.unlock();
    }
    return queue;
  }

  /**
   * Creates a new timestamp queue for odometry timing.
   *
   * @return A queue that will receive timestamps.
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.LOCK.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.LOCK.unlock();
    }
    return queue;
  }

  @SuppressWarnings("PMD.CyclomaticComplexity")
  @Override
  public void run() {
    while (true) {
      try {
        if (isCANFD && talonSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 * ODOMETRY_PERIOD.in(Seconds), talonSignals);
        } else {
          sleep(Math.round(ODOMETRY_PERIOD.in(Milliseconds)));
          if (talonSignals.length > 0) BaseStatusSignal.refreshAll(talonSignals);
        }
      } catch (Exception e) {
        e.printStackTrace();
      }

      Drive.LOCK.lock();

      try {
        // FPGA returns in microseconds (1000000 microseconds in a second)
        double timestamp = RobotController.getFPGATime() / 1e6;

        double totalLatency = 0.0;
        for (BaseStatusSignal signal : talonSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (talonSignals.length > 0) {
          timestamp -= totalLatency / talonSignals.length;
        }

        // add updates to queues
        for (int i = 0; i < talonSignals.length; i++) {
          talonQueues.get(i).offer(talonSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < otherSignals.size(); i++) {
          otherQueues.get(i).offer(otherSignals.get(i).getAsDouble());
        }
        for (Queue<Double> timestampQueue : timestampQueues) {
          timestampQueue.offer(timestamp);
        }
      } finally {
        Drive.LOCK.unlock();
      }
    }
  }
}
