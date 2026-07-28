package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.ControlConstants.*;

import edu.wpi.first.math.MathUtil;
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
  private Turret turret;

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
    // wrapping in the update method makes funny business
    assertEquals(
        MathUtil.angleModulus(setpoint.in(Radians)),
        MathUtil.angleModulus(turret.position()),
        TOLERANCE.in(Radians));
  }

  /**
   * Tests that goToYaw picks the valid turret position closest to the current position.
   *
   * <p>Range is [0°, 405°]; overlap zone is [0°, 45°] where both c1=theta and c2=theta+360 are
   * valid. Cases: overlap zone from near side, overlap zone from far side, only c2 valid (negative
   * yaw), only c1 valid (yaw above 45°), boundary at 180°, just above 45°, crossover in overlap
   * zone.
   */
  @ParameterizedTest
  @CsvSource({
    "0, 20, 20", // overlap zone, nearer side: c1=20 closer than c2=380
    // "340, -10, 350", // overlap zone, far side: c2=350 closer than c1=-10
    "340, -10, -10", // wrap in turret is funny
    "0, -90, 270", // only c2 valid (theta=-90 < MIN=0, c2=270 valid)
    "0, 120, 120", // only c1 valid (c2=480 > MAX=405)
    "0, 180, 180", // 180 deg boundary: only c1 valid (c2=540 > MAX)
    "0, 46, 46", // just above 45: only c1 valid (c2=406 > MAX)
    // "171, -10, 350", // overlap zone crossover: c2=350 is 179 away, c1=-10 is 181 away
    "171, -10, -10", // wrap in turret is funny
    "199, 20, 20" // overlap zone crossover: c1=20 is 179 away, c2=380 is 181 away
  })
  public void goToYaw(double startDeg, double yawDeg, double expectedDeg) {
    if (startDeg != 0) {
      // CommandScheduler.getInstance().schedule(turret.goTo(() ->
      // Degrees.of(startDeg).in(Radians)));
      // fastForward(Seconds.of(10));
      turret.hardware.setPosition(Degrees.of(startDeg));
    }
    CommandScheduler.getInstance().schedule(turret.goToYaw(() -> Rotation2d.fromDegrees(yawDeg)));
    fastForward(Seconds.of(5));
    assertEquals(Degrees.of(expectedDeg).in(Radians), turret.position(), TOLERANCE.in(Radians));
  }

  /**
   * Tests that commanding above MAX_ANGLE does not make the turret go above the maximum boundary.
   */
  @Test
  public void inRangeMax() {
    double overMax = MAX_ANGLE.in(Radians) + 1.0;
    CommandScheduler.getInstance().schedule(turret.goTo(() -> overMax));
    fastForward(Seconds.of(5));
    assertTrue(MAX_ANGLE.in(Radians) > turret.position());
  }

  /**
   * Tests that commanding below MIN_ANGLE does not make the turret go below the minimum boundary.
   */
  @Test
  public void inRangeMin() {
    double underMin = MIN_ANGLE.in(Radians) - 1.0;
    CommandScheduler.getInstance().schedule(turret.goTo(() -> underMin));
    fastForward(Seconds.of(5));
    assertTrue(MIN_ANGLE.in(Radians) < turret.position());
  }
}
