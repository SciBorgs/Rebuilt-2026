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
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;

public class ShotDataCollector {
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

  private static final AtomicBoolean RUNNING = new AtomicBoolean(false);

  protected int timestep;
  private BufferedWriter dataWriter;

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
          LoggingUtils.log("Shooting/DataLog TimeStep", timestep);
          LoggingUtils.log("Shooting/DataLog Status", RUNNING.get());
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
          Files.newBufferedWriter(
              Paths.get("resources/" + PATH + LOG_NAME + logIndex + ".ankit"),
              StandardCharsets.UTF_8);
    } catch (IOException exception) {
      exception.printStackTrace();
    }
  }

  /** Returns a command that begins data logging. */
  public Command startLogging() {
    return Commands.runOnce(
        () -> {
          DriverStation.reportWarning("Started Ankit Log!", false);
          RUNNING.set(true);
        });
  }

  /** Returns a command that ends data logging. */
  public Command endLogging() {
    return Commands.runOnce(
        () -> {
          DriverStation.reportWarning("Ended Ankit Log!", false);
          RUNNING.set(false);
        });
  }

  private void logTimestep() {
    if (!RUNNING.get() || dataWriter == null) return;

    try {
      dataWriter.write(
          indexer.blocked.getAsBoolean() + "," + shooter.getVelocity() + "," + hood.angle());
      dataWriter.newLine();
      timestep++;

      if (timestep > MAX_TIMESTEPS) {
        timestep = 0;

        executorService.shutdown();
        dataWriter.close();
      }
    } catch (IOException exception) {
      endLogging();
    }
  }
}
