package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.Test.*;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hood.SimHood;

public class HoodTest {

  private Hood hood;

  @BeforeEach
  public void initialize() {
    setupTests();
    hood = new Hood(new SimHood());
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(hood);
  }

  @Test
  public void maxAngle() {
    runUnitTest(hood.goToTest(MIN_ANGLE));
    runUnitTest(hood.goToTest(MAX_ANGLE));
  }

  @Test
  public void randAngle() {
    hood.goToTest(
        Radians.of(Math.random() * (MAX_ANGLE.minus(MIN_ANGLE).in(Radians))).plus(MIN_ANGLE));
  }
}
