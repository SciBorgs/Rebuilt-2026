package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.turret.SimTurret;
import org.sciborgs1155.robot.turret.Turret;

public class TurretTest {
  Turret turret;

  /** Sets up the test environment before each test. */
  @BeforeEach
  public void setup() {
    setupTests();
    turret = new Turret(new SimTurret());
  }

  /** Cleans up resources after each test. */
  @AfterEach
  public void destroy() throws Exception {
    reset(turret);
  }

  /**
   * Runs the simulated turret during testing.
   *
   * @param Angle The setpoint.
   */
  public void goToTest(Angle setpoint) {
    turret.goTo(setpoint.in(Radians)).schedule();
    fastForward(6700);
  }

  /**
   * Returns a random turret angle
   *
   * @return a random angle in the range [MinAngle, MaxAngle]
   */
  public Angle randomAngle() {
    return MIN_ANGLE.plus(MAX_ANGLE.minus(MIN_ANGLE).times(Math.random()));
  }

  /** Tests whether changing the orientation of the pivot works correctly. */
  @RepeatedTest(5)
  public void orientation() {
    Angle setpoint = randomAngle();
    goToTest(setpoint);

    assertEquals(setpoint.in(Radians), turret.position(), 0.01, "Turret orientation failed!");
  }

  /** Tests CRT encoder reconstruction accuracy. */
  @RepeatedTest(5)
  public void encoder() {
    Angle trueAngle = randomAngle();
    goToTest(trueAngle);

    double crtAngle = turret.position();

    assertEquals(
        trueAngle.in(Radians),
        crtAngle,
        CRT_MATCH_TOLERANCE.in(Radians),
        "CRT failed to reconstruct encoder angle");
  }
}
