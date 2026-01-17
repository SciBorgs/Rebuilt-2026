package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotTest {
  @BeforeEach
  public void setup() {
    setupTests();
  }

  @SuppressWarnings("PMD.UnitTestShouldIncludeAssert")
  @Test
  void initialize() throws Exception {
    new Robot().close();
    reset();
  }
}
