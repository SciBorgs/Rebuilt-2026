package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.climb.ClimbConstants.MIN_HEIGHT;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.climb.Climb;
import org.sciborgs1155.robot.climb.SimClimb;

public class ClimbTest {

  private Climb climb;

  /** initializes unit tests and sim climb */
  @BeforeEach
  public void initialize() {
    setupTests();
    climb = new Climb(new SimClimb());
  }

  /** resets the sim climb */
  @AfterEach
  public void destroy() {
    try {
      reset(climb);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /** test for climb to go to minimum height */
  @Test
  @SuppressWarnings("PMD.UnitTestShouldIncludeAssert")
  public void goDown() {
    runUnitTest(climb.goToTest(MIN_HEIGHT));
  }
}
