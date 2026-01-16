package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.hopper.Hopper;

public class HopperTest {
  Hopper hopper;

  @BeforeEach
  public void setup() {
    setupTests();
    hopper = Hopper.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(hopper);
  }

  @Test
  public void testHopper() {}
}
