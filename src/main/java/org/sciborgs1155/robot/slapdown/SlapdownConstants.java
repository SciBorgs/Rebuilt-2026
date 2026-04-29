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
import org.sciborgs1155.robot.Robot;

public class SlapdownConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double P = Robot.isReal() ? 6 : 10;
  public static final double I = 0;
  public static final double D = 0.13;

  public static final double S = 0.34;
  public static final double V = 0.85;
  public static final double G = 0.17;
  public static final double A = 0.62;

  public static final double EXTEND_VOLTAGE = -6.0; // down
  public static final double RETRACT_VOLTAGE = 6.0; // up
  public static final double KEEP_US_UP_VOLTAGE = 1.0;

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.3).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.7);
  public static final Time TIME_OUT = Seconds.of(30);

  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(5);
  public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(8);
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

  // 4-bar, old
  // public static final DCMotor GEARBOX = DCMotor.getKrakenX44(1);
  // public static final double GEARING = 27;
  // public static final double MOI = 0.2135396026; // moment of inertia
  // public static final Distance LENGTH = Inches.of(18.5);
  // public static final Angle MIN_ANGLE = Degrees.of(9.7);
  // public static final Angle MAX_ANGLE = Degrees.of(83.7);
  // public static final Angle START_ANGLE = MAX_ANGLE;

  // new slapdown
  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  public static final double GEARING = 25.0 * 2.5; // ratio of gearing
  public static final double MOI = 0.2135396026; // moment of inertia
  public static final Distance LENGTH = Inches.of(18.5);
  public static final Angle MIN_ANGLE = Radians.of(0.0); // 0.0
  public static final Angle MAX_ANGLE = Radians.of(1.864 - 0.07); // 1.864 // 2.04
  public static final Angle START_ANGLE = MAX_ANGLE;

  public static final Angle POSITION_TOLERANCE = Radians.of(0.07);

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.5);

  public static final Angle SQUEEZE_RETRACT = Degrees.of(0);
  public static final Angle SQUEEZE_EXTEND = Degrees.of(60);
  public static final double SQUEEZE_VOLTS = 2.0;
  public static final double STALLING_CURRENT = 40.0; // if over, motor is stalling
}
