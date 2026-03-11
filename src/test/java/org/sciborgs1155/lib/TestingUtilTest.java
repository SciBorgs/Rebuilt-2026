package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public class TestingUtilTest {
  int x;

  /** Sets up the test environment and initializes x to 0 before each test. */
  @BeforeEach
  public void setup() {
    setupTests();
    x = 0;
  }

  /** Resets the test environment and x to 0 after each test. */
  @AfterEach
  public void clear() throws Exception {
    reset();
    x = 0;
  }

  private void increment() {
    x += 1;
  }

  private void set(int x) {
    this.x = x;
  }

  /** Tests that the driver station is enabled during tests. */
  @org.junit.jupiter.api.Test
  public void enabled() {
    assertTrue(DriverStation.isEnabled());
  }

  /**
   * Tests that runToCompletion correctly advances simulation time.
   *
   * @param timeout The timeout duration to test with.
   */
  @ParameterizedTest
  @ValueSource(doubles = {0.4, 2, 3.2, 4.03})
  public void runToCompletionTest(double timeout) {
    Command c = Commands.run(() -> {}).withTimeout(timeout);
    double startTime = Timer.getFPGATimestamp();
    runToCompletion(c);
    assertEquals(timeout, Timer.getFPGATimestamp() - startTime, 0.3);
  }

  /**
   * Asserts that the fault counts match the expected values.
   *
   * @param infoCount Expected number of info faults.
   * @param warningCount Expected number of warning faults.
   * @param errorCount Expected number of error faults.
   */
  public void assertFaultCount(int infoCount, int warningCount, int errorCount) {
    FaultLogger.update();
    Set<Fault> faults = FaultLogger.totalFaults();
    Set<Fault> infos =
        faults.stream().filter(f -> f.type() == FaultType.INFO).collect(Collectors.toSet());
    Set<Fault> warnings =
        faults.stream().filter(f -> f.type() == FaultType.WARNING).collect(Collectors.toSet());
    Set<Fault> errors =
        faults.stream().filter(f -> f.type() == FaultType.ERROR).collect(Collectors.toSet());
    assertEquals(infoCount, infos.size(), infos.toString());
    assertEquals(warningCount, warnings.size());
    assertEquals(errorCount, errors.size());
  }
}
