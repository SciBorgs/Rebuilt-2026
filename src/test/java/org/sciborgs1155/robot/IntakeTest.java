package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.intake.Intake;

public class IntakeTest {
  Intake intake;

  final double TOLERANCE = 0.1;

  @BeforeEach
  public void setup() {
    setupTests();
    intake = Intake.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(intake);
  }

  @Test
  public void extend() {
    intake.extend();
  }

  @Test
  public void retract() {
    intake.retract();
  }
}
