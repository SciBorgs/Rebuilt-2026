package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.FaultLogger.FaultType;

public sealed interface Assertion {
  /**
   * Applies this assertion, either in a unit test context or a runtime context.
   *
   * @param unitTest Whether this is being run in a unit test.
   */
  void apply(boolean unitTest);

  /** Asserts that a condition is true and reports to FaultLogger. */
  private static void reportTrue(boolean condition, String faultName, String description) {
    FaultLogger.report(
        faultName,
        (condition ? "success! " : "") + description,
        condition ? FaultType.INFO : FaultType.WARNING);
  }

  /**
   * Asserts that two values are equal (with some tolerance) and reports to FaultLogger.
   *
   * @param delta tolerance
   */
  private static void reportEquals(String faultName, double expected, double actual, double delta) {
    reportTrue(
        Math.abs(expected - actual) <= delta,
        faultName,
        "expected: " + expected + "; actual: " + actual);
  }

  record TruthAssertion(BooleanSupplier condition, String faultName, Supplier<String> description)
      implements Assertion {
    @Override
    public void apply(boolean unitTest) {
      if (unitTest) {
        assertTrue(condition, faultName + ": " + description.get());
      } else {
        reportTrue(condition.getAsBoolean(), faultName, description.get());
      }
    }
  }

  record EqualityAssertion(
      String faultName, DoubleSupplier expected, DoubleSupplier actual, double delta)
      implements Assertion {
    @Override
    public void apply(boolean unitTest) {
      if (unitTest) {
        assertEquals(expected.getAsDouble(), actual.getAsDouble(), delta, faultName);
      } else {
        reportEquals(faultName, expected.getAsDouble(), actual.getAsDouble(), delta);
      }
    }
  }

  /**
   * @return a truth assertion
   */
  static TruthAssertion tAssert(
      BooleanSupplier condition, String faultName, Supplier<String> description) {
    return new TruthAssertion(condition, faultName, description);
  }

  /**
   * @return an equality assertion
   */
  static EqualityAssertion eAssert(
      String faultName, DoubleSupplier expected, DoubleSupplier actual, double delta) {
    return new EqualityAssertion(faultName, expected, actual, delta);
  }

  /**
   * @return an equality assertion
   */
  static EqualityAssertion eAssert(
      String faultName, DoubleSupplier expected, DoubleSupplier actual) {
    return eAssert(faultName, expected, actual, 0);
  }
}
