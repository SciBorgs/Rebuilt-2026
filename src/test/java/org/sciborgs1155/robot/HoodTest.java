package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.Test.*;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;

import edu.wpi.first.units.measure.Angle;
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
    Angle randAngle =
        Radians.of(Math.random() * MAX_ANGLE.minus(MIN_ANGLE).in(Radians)).plus(MIN_ANGLE);
    hood.goTo(randAngle);
    fastForward(Seconds.of(.5));
    assertTrue(hood.atPosition(randAngle.in(Radians)));
  }
}
