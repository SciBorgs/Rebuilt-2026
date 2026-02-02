package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.climb.ClimbConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.climb.ClimbConstants.MIN_HEIGHT;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.climb.Climb;

public class ClimbTest {

  Climb climb;

  /** initializes unit tests and sim climb */
  @BeforeEach
  public void initialize() {
    setupTests();
    climb = Climb.create();
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
    CommandScheduler.getInstance()
        .schedule(climb.goTo(MIN_HEIGHT.in(Meters)).withDeadline(Commands.waitSeconds(3)));
    assert climb.atPosition(MIN_HEIGHT.in(Meters));
  }

  /** test for climb to go to minimum height */
  @Test
  @SuppressWarnings("PMD.UnitTestShouldIncludeAssert")
  public void goUp() {
    climb.goTo(MAX_HEIGHT.in(Meters)).withDeadline(Commands.waitSeconds(3)).execute();
    assert climb.atPosition(MAX_HEIGHT.in(Meters));
  }
}
