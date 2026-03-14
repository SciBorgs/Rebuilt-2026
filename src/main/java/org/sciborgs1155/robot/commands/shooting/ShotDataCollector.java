package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;

public class ShotDataCollector implements AutoCloseable {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Hood hood;

  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();

  private static final double ANALYSIS_PERIOD = 0.004;
  private static final double INITIAL_DELAY = 0;

  private static final String PATH = "shooting/dataLogs/";
  private static final String LOG_NAME = "Log";

  private static final long MAX_LOG = 500;
  private static final long MAX_TIMESTEPS = Math.round(300 / ANALYSIS_PERIOD);

  private final AtomicBoolean running = new AtomicBoolean(false);

  protected final AtomicInteger timestep = new AtomicInteger(0);
  private Optional<BufferedWriter> dataWriter = Optional.empty();
  private final AtomicReference<CachedState> cachedState =
      new AtomicReference<>(new CachedState(false, 0.0, 0.0));

  // Snapshot of subsystem state written on the main thread, read on the executor thread.
  private record CachedState(boolean blocked, double velocity, double angle) {}

  /** Creates a ShotDataCollector that logs shooter data to a file. */
  public ShotDataCollector(Shooter shooter, Indexer indexer, Hood hood) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.hood = hood;

    int logIndex = 1;
    while (logIndex < MAX_LOG) {
      if (Paths.get("resources/" + PATH + LOG_NAME + logIndex + ".ankit").toFile().exists())
        logIndex++;
      else break;
    }

    executorService.scheduleAtFixedRate(
        () -> {
          LoggingUtils.log("Shooting/DataLog TimeStep", timestep.get());
          LoggingUtils.log("Shooting/DataLog Status", running.get());
        },
        Math.round(INITIAL_DELAY * 1000),
        Math.round(Constants.PERIOD.in(Milliseconds)),
        TimeUnit.MILLISECONDS);

    executorService.scheduleAtFixedRate(
        this::logTimestep,
        Math.round(INITIAL_DELAY * 1000),
        Math.round(ANALYSIS_PERIOD * 1000),
        TimeUnit.MILLISECONDS);

    try {
      dataWriter =
          Optional.of(
              Files.newBufferedWriter(
                  Paths.get("resources/" + PATH + LOG_NAME + logIndex + ".ankit"),
                  StandardCharsets.UTF_8));
    } catch (IOException exception) {
      DriverStation.reportError(
          "ShotDataCollector: failed to open log file: " + exception.getMessage(), false);
    }
  }

  /**
   * Caches the current subsystem state for use by the background logging thread. Call this once per
   * main robot loop iteration.
   */
  public void update() {
    cachedState.set(
        new CachedState(indexer.blocked.getAsBoolean(), shooter.velocity(), hood.angle()));
  }

  /** Returns a command that begins data logging. */
  public Command startLogging() {
    return Commands.runOnce(
        () -> {
          DriverStation.reportWarning("Started Ankit Log!", false);
          running.set(true);
        });
  }

  /** Returns a command that ends data logging. */
  public Command endLogging() {
    return Commands.runOnce(
        () -> {
          DriverStation.reportWarning("Ended Ankit Log!", false);
          running.set(false);
        });
  }

  private void logTimestep() {
    if (!running.get() || dataWriter.isEmpty()) return;

    BufferedWriter writer = dataWriter.get();
    CachedState state = cachedState.get();

    try {
      writer.write(state.blocked() + "," + state.velocity() + "," + state.angle());
      writer.newLine();

      if (timestep.incrementAndGet() > MAX_TIMESTEPS) {
        timestep.set(0);
        executorService.shutdown();
        writer.close();
        dataWriter = Optional.empty();
      }
    } catch (IOException exception) {
      running.set(false);
    }
  }

  @Override
  public void close() {
    executorService.shutdownNow();
    dataWriter.ifPresent(
        writer -> {
          try {
            writer.close();
          } catch (IOException exception) {
            DriverStation.reportError(
                "ShotDataCollector: failed to close log file: " + exception.getMessage(), false);
          }
        });
    dataWriter = Optional.empty();
  }
}
