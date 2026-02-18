package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.turret.Turret;

public class TurretTest {
  private Turret turret;

  /** Sets up the test environment before each test. */
  @BeforeEach
  public void setup() {
    setupTests();
    turret = Turret.create();
  }

  /** Cleans up resources after each test. */
  @AfterEach
  public void destroy() throws Exception {
    reset(turret);
  }

  /**
   * Returns a random turret angle
   *
   * @return a random angle in the range [MinAngle, MaxAngle]
   */
  public Angle randomAngle() {
    return MIN_ANGLE.plus(MAX_ANGLE.minus(MIN_ANGLE).times(Math.random()));
  }

  /** Tests whether changing the orientation of the turret works correctly. */
  @RepeatedTest(5)
  public void orientation() {
    Angle setpoint = randomAngle();

    CommandScheduler.getInstance()
        .schedule(turret.goTo(() -> setpoint.in(Radians)).withDeadline(Commands.waitSeconds(3)));
    fastForward(Seconds.of(3));
    assert turret.atGoal();
  }
}
