package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.slapdown.SimSlapdown;
import org.sciborgs1155.robot.slapdown.Slapdown;

public class SlapdownTest {
  private Slapdown slapdown;

  /** make the slapdown */
  @BeforeEach
  public void setup() {
    slapdown = Slapdown.create();
    setupTests();
  }

  /**
   * @throws Exception reset the position
   */
  @AfterEach
  public void destroy() throws Exception {
    reset(slapdown);
  }

  /** make the arm move */
  @Test
  public void setArmVoltage() {
    slapdown = new Slapdown(new SimSlapdown());

    runUnitTest(slapdown.goToTest(MAX_ANGLE.in(Radians)));
  }
}
