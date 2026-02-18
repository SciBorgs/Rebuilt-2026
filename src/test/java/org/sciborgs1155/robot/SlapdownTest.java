package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.MIN_ANGLE;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.slapdown.Slapdown;

public class SlapdownTest {
  private Slapdown slapdown;
  private final Random random = new Random();

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

  /** Tests whether changing the orientation of the slapdown works correctly. */
  @RepeatedTest(5)
  public void orientation() {
    double setpoint = random.nextDouble(MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    CommandScheduler.getInstance()
        .schedule(slapdown.goTo(setpoint).withDeadline(Commands.waitSeconds(3)));
    fastForward(Seconds.of(3));
    assert slapdown.atPosition(setpoint);
  }
}
