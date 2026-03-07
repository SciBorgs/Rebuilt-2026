package org.sciborgs1155.robot.commands.shooting;

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
import org.sciborgs1155.robot.shooter.Shooter;

public class ShooterAnalyzer {
  private final Shooter shooter;
  private final ScheduledExecutorService executorService =
      Executors.newSingleThreadScheduledExecutor();
  private static final double ANALYSIS_PERIOD = 0.05;
  private static final String FILE_NAME = "ShooterData";

  private BufferedWriter fileWriter;

  public ShooterAnalyzer(Shooter shooter) {
    this.shooter = shooter;

    try {
      fileWriter =
          Files.newBufferedWriter(
              Paths.get("resources/shooting/" + FILE_NAME + ".ankit"), StandardCharsets.UTF_8);
    } catch (IOException exception) {
      exception.printStackTrace();
      fileWriter = null;
    }
  }

  public Command startLogging() {
    return Commands.runOnce(
        () ->
            executorService.scheduleAtFixedRate(
                this::logData, 0, Math.round(ANALYSIS_PERIOD * 1000), TimeUnit.MILLISECONDS));
  }

  public Command endLogging() {
    return Commands.runOnce(
        () -> {
          try {
            fileWriter.close();
          } catch (IOException exception) {
            exception.printStackTrace();
          }

          executorService.shutdown();
        });
  }

  private void logData() {
    try {
      fileWriter.write(shooter.getVelocity() + ", ");
    } catch (IOException exception) {
      exception.printStackTrace();
    }
  }
}
