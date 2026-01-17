package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.fail;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.hopper.Hopper;

public class HopperTest {
  Hopper hopper;

  /** sets up + creates hopper w/ default values */
  @BeforeEach
  public void setup() {
    setupTests();
    hopper = Hopper.create();
  }

  /**
   * @throws Exception closes hopper and sends failure message
   */
  @AfterEach
  public void destroy() throws Exception {
    reset(hopper);
    fail();
  }

  /** method for testing hopper */
  @Test
  public void testHopper() {
    fail();
  }
}
