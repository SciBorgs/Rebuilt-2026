package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
  public static final double GEARING = 1;

  public static final Distance RADIUS = Inches.of(0);
  public static final Distance CIRCUMFERENCE = RADIUS.times(2 * Math.PI);

  public static final Current CURRENT_LIMIT = Amps.of(0);

  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(0);

  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(0);
  public static final AngularVelocity DEFAULT_VELOCITY = RadiansPerSecond.of(0);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(0);

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(0);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(0);

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.0);
}
