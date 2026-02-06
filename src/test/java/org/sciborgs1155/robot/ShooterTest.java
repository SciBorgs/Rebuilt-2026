package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.shooter.Shooter;

public class ShooterTest {
  private Shooter shooter;

  /** Sets up the tests */
  @BeforeEach
  public void setup() {
    setupTests();
    shooter = Shooter.create();
  }

  /**
   * Destroys the test.
   *
   * @throws Exception
   */
  @AfterEach
  public void destroy() throws Exception {
    reset(shooter);
  }

  /**
   * Tests a random velocity for the shooter wheel.
   *
   * @throws Exception
   */
  @RepeatedTest(5)
  public void randVelocity() throws Exception {
    double val = Math.random();
    runUnitTest(shooter.goToTest(() -> val * MAX_VELOCITY.in(RadiansPerSecond)));
  }
}
