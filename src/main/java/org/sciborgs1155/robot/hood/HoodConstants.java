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
import org.sciborgs1155.robot.Robot;

public class HoodConstants {
  public static final Angle MIN_ANGLE = Degrees.of(15);
  public static final Angle MAX_ANGLE = Degrees.of(53);
  public static final Mass MASS = Pounds.of(1.307);
  public static final Angle STARTING_ANGLE = MIN_ANGLE;
  public static final Current SUPPLY_LIMIT = Amps.of(30);
  public static final Current STATOR_LIMIT = Amps.of(30);
  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(20);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(20);
  public static final Angle POSITION_TOLERANCE = Radians.of(0.01);
  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.01);
  public static final Angle SHOOTING_ANGLE_OFFSET = Degrees.of(90);

  // Sysid constants
  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.5).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.5);
  public static final Time TIME_OUT = Seconds.of(6);

  public static final Distance HOOD_RADIUS = Inches.of(9.29);
  public static final double MOI = 0.0045821517; // kg*m^2

  public static final Angle DEFAULT_ANGLE = STARTING_ANGLE;
  public static final double GEARING = 95.33333;

  public class PID {
    public static final double P = Robot.isReal() ? 20 : 10;
    public static final double I = Robot.isReal() ? 0 : 0.000001;
    public static final double D = Robot.isReal() ? 0.2 : 0.000001;
    public static final double S = 0.295;
    public static final double V = .7;
    public static final double G = .015;
    public static final double A = 0;
  }
}
