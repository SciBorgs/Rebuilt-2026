package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {

  public static final Angle MIN_ANGLE = Degrees.of(15);
  public static final Angle MAX_ANGLE = Degrees.of(53);
  public static final Mass MASS = Pounds.of(1.307);
  public static final Angle STARTING_ANGLE = MIN_ANGLE;
  public static final Current SUPPLY_LIMIT = Amps.of(30);
  public static final Current STATOR_LIMIT = Amps.of(30);
  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(1);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(1);
  public static final Angle POSITION_TOLERANCE = Radians.of(0.01);
  public static final Angle SHOOTING_ANGLE_OFFSET = Degrees.of(90);

  // Sysid constants
  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.5).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.3);
  public static final Time TIME_OUT = Seconds.of(3);

  public static final Distance HOOD_RADIUS = Inches.of(9.29);
  public static final double MOI = 0.0045821517; // kg*m^2

  public static final Angle DEFAULT_ANGLE = STARTING_ANGLE;
  public static final double GEARING = 12.0 / 44.0 * 18.0 / 14.0 * 182.0 / 10.0;

  public class PID {
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0;
    public static final double V = 0;
    public static final double G = 0;
    public static final double A = 0;
  }
}
