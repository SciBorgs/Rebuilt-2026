package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.ControlConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.sciborgs1155.robot.turret.Turret;

public class TurretTest {
  Turret turret;

  /** Sets up the test environment and initializes turret before each test. */
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

  /** Generates a random angle between MIN_ANGLE and MAX_ANGLE */
  private Angle randomAngle() {
    return MIN_ANGLE.plus(MAX_ANGLE.minus(MIN_ANGLE).times(Math.random()));
  }

  /** Tests that the turret reaches a random valid angle within tolerance. */
  @RepeatedTest(5)
  public void orientation() {
    Angle setpoint = randomAngle();
    CommandScheduler.getInstance().schedule(turret.goTo(() -> setpoint.in(Radians)));
    fastForward(Seconds.of(5));
    assertEquals(setpoint.in(Radians), turret.position(), TOLERANCE.in(Radians));
  }

  /**
   * Tests that goToYaw picks the valid turret position closest to the current position.
   *
   * Cases: overlap zone from near side (start 0, yaw 45, expect 45), overlap zone from far side
   * (start -350, yaw 45, expect -315), only theta valid (start 0, yaw -90, expect -90), only theta
   * minus 360 valid (start 0, yaw 120, expect -240).
   */
  @ParameterizedTest
  @CsvSource({
    "0, 45, 45",      // overlap zone, nearer side: c1 closer
    "-350, 45, -315", // overlap zone, far side: c2 closer
    "0, -90, -90",    // only c1 valid (negative angle)
    "0, 120, -240",   // only c2 valid (above 90)
    "0, 180, -180",   // 180 deg boundary: only c2 valid, c2 = -180
    "0, 91, -269",    // just above 90: only c2 valid
    "-181, 0, -360",  // overlap zone crossover: c2=-360 is 179 away, c1=0 is 181 away
    "-179, 0, 0"      // overlap zone crossover: c1=0 is 179 away, c2=-360 is 181 away
  })
  public void goToYaw(double startDeg, double yawDeg, double expectedDeg) {
    if (startDeg != 0) {
      CommandScheduler.getInstance().schedule(turret.goTo(() -> Degrees.of(startDeg).in(Radians)));
      fastForward(Seconds.of(10));
    }
    CommandScheduler.getInstance().schedule(turret.goToYaw(() -> Rotation2d.fromDegrees(yawDeg)));
    fastForward(Seconds.of(5));
    assertEquals(Degrees.of(expectedDeg).in(Radians), turret.position(), TOLERANCE.in(Radians));
  }

  /** Tests that commanding above MAX_ANGLE clamps the turret to the maximum boundary. */
  @Test
  public void clampedAtMax() {
    double overMax = MAX_ANGLE.in(Radians) + 1.0;
    CommandScheduler.getInstance().schedule(turret.goTo(() -> overMax));
    fastForward(Seconds.of(5));
    assertEquals(MAX_ANGLE.in(Radians), turret.position(), TOLERANCE.in(Radians));
  }

  /** Tests that commanding below MIN_ANGLE clamps the turret to the minimum boundary. */
  @Test
  public void clampedAtMin() {
    double underMin = MIN_ANGLE.in(Radians) - 1.0;
    CommandScheduler.getInstance().schedule(turret.goTo(() -> underMin));
    fastForward(Seconds.of(5));
    assertEquals(MIN_ANGLE.in(Radians), turret.position(), TOLERANCE.in(Radians));
  }
}
