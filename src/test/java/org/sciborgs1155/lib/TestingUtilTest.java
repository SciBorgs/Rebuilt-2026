package org.sciborgs1155.lib;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.lib.Assertion.tAssert;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.Test.toCommand;
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
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Assertion.TruthAssertion;
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
  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
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

  /** Tests creating a Test from a Command and running it to completion. */
  @org.junit.jupiter.api.Test
  public void fromCommandTest() {
    assertEquals(0, x);
    Test t = Test.fromCommand(runOnce(this::increment));
    Command c = toCommand(t);
    runToCompletion(c);
    assertEquals(1, x);
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

  /**
   * Tests the system check functionality with passing and failing assertions.
   *
   * @param x The value to test with.
   */
  @ParameterizedTest
  @ValueSource(ints = {-4, 3, 9})
  public void systemCheckTest(int x) {
    EqualityAssertion goodAssertion = eAssert("x", () -> x, () -> this.x);
    Test passes = new Test(runOnce(() -> set(x)), Set.of(goodAssertion));
    runToCompletion(toCommand(passes));
    assertFaultCount(1, 0, 0);

    TruthAssertion badAssertion = tAssert(() -> x != this.x, "x", () -> "fails");
    Test fails = new Test(runOnce(() -> set(x)), Set.of(badAssertion));
    runToCompletion(toCommand(fails));
    assertFaultCount(1, 1, 0);

    FaultLogger.clear();
    FaultLogger.unregisterAll();
    Test combo = new Test(runOnce(() -> set(x)), Set.of(goodAssertion, badAssertion));
    runToCompletion(toCommand(combo));
    assertFaultCount(1, 1, 0);
  }

  /**
   * Tests the unit test functionality with passing and failing assertions.
   *
   * @param x The value to test with.
   */
  @ParameterizedTest
  @ValueSource(ints = {-4, 3, 9})
  public void unitTestTest(int x) {
    EqualityAssertion goodAssertion = eAssert("x", () -> x, () -> this.x);
    Test passes = new Test(runOnce(() -> set(x)), Set.of(goodAssertion));
    runUnitTest(passes);
    assertFaultCount(0, 0, 0);

    TruthAssertion badAssertion = tAssert(() -> x != this.x, "x", () -> "fails");
    Test fails = new Test(runOnce(() -> set(x)), Set.of(badAssertion));
    assertThrows(AssertionError.class, (Executable) () -> runUnitTest(fails));
    assertFaultCount(0, 0, 0);
  }
}
