package org.sciborgs1155.robot;

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

  final double TOLERANCE = 0.1;

  @BeforeEach
  /** make the slapdown */
  public void setup() {
    slapdown = Slapdown.create();
    setupTests();
  }

  @AfterEach
  /**
   * @throws Exception reset the position
   */
  public void destroy() throws Exception {
    reset(slapdown);
  }

  @Test
  /** make the arm move */
  public void setArmVoltage() {

    System.out.println(Robot.isReal());
    System.out.println("something sucks");

    slapdown = new Slapdown(new SimSlapdown());

    System.out.println(slapdown.getSubsystem());

    runUnitTest(slapdown.goToTest(MAX_ANGLE));
  }
}
