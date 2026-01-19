package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

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

  /** Tests whether changing the orientation of the pivot works correctly. */
  @RepeatedTest(5)
  public void orientation() {
    Angle setpoint = Radians.of(Math.random() * Math.PI * 2).minus(Radians.of(Math.PI));

    turret.setAngle(setpoint);
    run(turret.run());

    fastForward(10000);
    assertEquals(
        setpoint.in(Radians), turret.position().in(Radians), 0.01, "Turret orientation failed!");
  }
}
