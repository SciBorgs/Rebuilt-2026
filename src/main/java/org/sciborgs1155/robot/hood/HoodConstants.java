package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {

  public static final Angle MIN_ANGLE = Degrees.of(14.196795).plus(Degrees.of(90));
  public static final Angle MAX_ANGLE = Degrees.of(75.8).plus(MIN_ANGLE).plus(Degrees.of(90));
  public static final double MOI = 1;
  public static final Angle STARTING_ANGLE = MIN_ANGLE;
  public static final Current SUPPLY_LIMIT = Amps.of(30);
  public static final Current STATOR_LIMIT = Amps.of(30);
  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(2);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(2);

  public static final Angle POS_TOLERANCE = Radians.of(0.01);

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.5).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.3);
  public static final Time TIME_OUT = Seconds.of(3);

  public static final double K_P = 10;
  public static final double K_I = 0;
  public static final double K_D = 1;
  public static final double K_S = 0;
  public static final double K_V = 0;
  public static final double K_G = 0;
  public static final double K_A = 0;

  public static final double HOOD_RADIUS = 8.4;
  public static final Angle DEFAULT_ANGLE = MIN_ANGLE;
  public static final double GEARING = 1.0 / 8;
}
