package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import org.sciborgs1155.robot.Robot;

public class ShooterConstants {
  public static final double GEARING = 1 / 0.8;

  public static final Distance RADIUS = Inches.of(2);
  public static final Distance CIRCUMFERENCE = RADIUS.times(2 * Math.PI);
  public static final double MOI = 0.0015328465;

  public static final double MAX_VOLTAGE = 12.0;

  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(0);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(400);

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(100);

  public static final Transform3d CENTER_TO_SHOOTER =
      new Transform3d(5.975, -5.975, -13.375, new Rotation3d());

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(3);

  public static final class ControlConstants {
    public static final double P = Robot.isReal() ? 0.0 : 0.070000;
    public static final double I = Robot.isReal() ? 0.0 : 0.000000001;
    public static final double D = Robot.isReal() ? 0.0 : 0.000000001;

    public static final double S = Robot.isReal() ? 0.0 : 0.000000001;
    public static final double V = Robot.isReal() ? 0.15318 : 0.023880;
    public static final double A = Robot.isReal() ? 0.066883 : 0.001000;
  }
}
