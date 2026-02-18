package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class SlapdownConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double P = 10;
  public static final double I = 0;
  public static final double D = 0.5;

  public static final double S = 0;
  public static final double V = 0;
  public static final double G = 0.7;
  public static final double A = 0;

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.5).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.3);
  public static final Time TIME_OUT = Seconds.of(1);

  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(5);
  public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(7);
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

  public static final DCMotor GEARBOX = DCMotor.getKrakenX44(1);
  public static final double GEARING = 27; // ratio of gearing
  public static final double MOI = 0.2135396026; // moment of inertia
  public static final Distance LENGTH = Inches.of(18.5);
  public static final Angle MIN_ANGLE = Degrees.of(9.7);
  public static final Angle MAX_ANGLE = Degrees.of(83.7);
  public static final Angle START_ANGLE = MAX_ANGLE;

  public static final Angle POSITION_TOLERANCE = Degrees.of(3);
}
