package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hood.SimHood;

public class HoodTest {

  private Hood hood;

  /** initializes unit tests and sim hood */
  @BeforeEach
  public void initialize() {
    setupTests();
    hood = new Hood(new SimHood());
  }

  /** resets the sim hood */
  @AfterEach
  public void destroy() throws Exception {
    reset(hood);
  }

  /** test for hood to go to random angles */
  @RepeatedTest(5)
  public void randAngle() {
    runUnitTest(
        hood.goToTest(
            Radians.of(Math.random() * (MAX_ANGLE.minus(MIN_ANGLE).in(Radians)))
                .plus(MIN_ANGLE)
                .plus(Radians.of(Math.PI / 2))));
  }
}
