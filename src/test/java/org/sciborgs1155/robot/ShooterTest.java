package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.SimWheel;

public class ShooterTest {
  private Shooter shooter;

  @BeforeEach
  public void setup() {
    setupTests();
    shooter = new Shooter(new SimWheel());
  }

  @AfterEach
  public void destory() throws Exception {
    reset(shooter);
  }

  @Test
  public void randVelocity() {
    runUnitTest(
        shooter.goToTest(RadiansPerSecond.of(Math.random() * MAX_VELOCITY.in(RadiansPerSecond))));
  }
}
