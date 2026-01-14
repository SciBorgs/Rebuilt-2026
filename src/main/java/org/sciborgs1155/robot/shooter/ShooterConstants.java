package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShooterConstants {
  public static final double GEARING = 1;

  public static final Distance RADIUS = Inches.of(0);
  public static final Distance CIRCUMFERENCE = RADIUS.times(2 * Math.PI);

  public static final Current CURRENT_LIMIT = null;

  public static final LinearAcceleration MAX_ACCEL = null;

  public static final AngularVelocity IDLE_VELOCITY = null;
  public static final AngularVelocity DEFAULT_VELOCITY = null;
  public static final LinearVelocity MAX_VELOCITY = null;

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.0);
}
