package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class HopperConstants {

  public static final Current CURRENT_LIMIT = Amps.of(30);
  public static final double INTAKING_VELOCITY = 1;

  public static final double P = 0.1;
  public static final double I = 0.0;
  public static final double D = 0.0;

  public static final double S = 0.2;
  public static final double V = 0.09;
  public static final double A = 0.0;

  public static final double MAX_VOLTAGE = 9;

  public static final Distance BIG_WHEEL_RADIUS = Inches.of(1.5);

  public static final LinearVelocity PASSTHROUGH_SPEED = MetersPerSecond.of(1.6);

  public static final double RADIANS_PER_SEC =
      PASSTHROUGH_SPEED.in(MetersPerSecond) / BIG_WHEEL_RADIUS.in(Meters);

  public static final double GEARING = 5.0;
}
