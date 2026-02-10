package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class SlapdownConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double P = 10;
  public static final double I = 0;
  public static final double D = 0;

  public static final double S = 0;
  public static final double V = 1;
  public static final double G = 0;
  public static final double A = 0;

  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(5);
  public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(7);
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  public static final double GEARING = 5; //ratio of gearing
  public static final double MOI = 5; //moment of inertia
  public static final Distance LENGTH = Inches.of(5);
  public static final Angle MIN_ANGLE = Radians.of(0);
  public static final Angle MAX_ANGLE = Radians.of(Math.PI * 1 / 2);
  public static final Angle START_ANGLE = Radians.of(0);

  public static final Angle POSITION_TOLERANCE = Radians.of(0.05);
}
