package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

  public static final AngularVelocity IDLE_VELOCITY = RPM.of(0);
  public static final AngularVelocity DEFAULT_VELOCITY = RPM.of(0);
  public static final AngularVelocity MAX_VELOCITY = RPM.of(0);

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(0);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(0);

  public static final Transform3d CENTER_TO_SHOOTER =
      new Transform3d(5.975, -5.975, -13.375, new Rotation3d());

  public static final class ControlConstants {
    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.0;
    public static final double A = 0.0;
  }

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.0);
}
