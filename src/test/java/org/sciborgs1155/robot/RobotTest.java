package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotTest {
  /** Sets up the test environment before each test. */
  @BeforeEach
  public void setup() {
    setupTests();
  }

  @Test
  @SuppressWarnings(
      "PMD.UnitTestShouldIncludeAssertion") // test is to make sure these methods do not error. no
  // assertion needed
  void initialize() throws Exception {
    new Robot().close();
    reset();
    assertTrue(true);
  }
}
