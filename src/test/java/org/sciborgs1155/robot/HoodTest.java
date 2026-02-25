package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.sciborgs1155.robot.hood.Hood;

public class HoodTest {

  private Hood hood;

  /** initializes unit tests and sim hood */
  @BeforeEach
  public void initialize() {
    setupTests();
    hood = Hood.create();
  }

  /** resets the sim hood */
  @AfterEach
  public void destroy() throws Exception {
    reset(hood);
  }

  // /** test for hood to go to random angles */
  // @Test
  // public void randAngle() {
  //   runUnitTest(
  //       hood.goToTest(
  //           Radians.of(Math.random() * MAX_ANGLE.minus(MIN_ANGLE).in(Radians)).plus(MIN_ANGLE)));
  // }
}
