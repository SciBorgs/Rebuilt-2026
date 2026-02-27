package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
  public static final double GEARING = 1 / 0.8;

  public static final Distance RADIUS = Inches.of(2);
  public static final Distance CIRCUMFERENCE = RADIUS.times(2 * Math.PI);

  public static final double MAX_VOLTAGE = 12.0;

  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(0);
  public static final AngularVelocity MAX_VELOCITY = RPM.of(7230);

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

  public static final Transform3d CENTER_TO_SHOOTER =
      new Transform3d(5.975, -5.975, -13.375, new Rotation3d());

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(1);

  public static final class ControlConstants {
    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.96;
    public static final double A = 0.42;
  }
}
