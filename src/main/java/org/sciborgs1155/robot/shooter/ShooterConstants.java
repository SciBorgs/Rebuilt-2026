package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
  public static final double GEARING = 1 / 0.8;

  public static final Distance RADIUS = Inches.of(2);
  public static final Distance CIRCUMFERENCE = RADIUS.times(2 * Math.PI);
  public static final double MOI = 0.0015328465; // kg*m^2

  public static final double MAX_VOLTAGE = 12.0;

  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(0);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(400);
  public static final double MAX_ACCELERATION = 5000;
  public static final double MAX_JERK = 2000;

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(100);

  public static final Transform3d CENTER_TO_SHOOTER =
      new Transform3d(Inches.of(5.975), Inches.of(5.975), Inches.of(13.375), new Rotation3d());

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(5);

  public static final class ControlConstants {
    public static final double P = 0.01;
    public static final double I = 0.0;
    public static final double D = 0.000003;

    public static final double S = 0.19071;
    public static final double V = 0.023602;
    public static final double A = 0.0024145;
  }
}
