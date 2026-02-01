package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class SlapdownConstants {
  public static final Current CURRENT_LIMIT = Amps.of(5);

  public static final double ROLLER_VOLTAGE = 10;
  public static final double EXTEND_VOLTAGE = 10;

  public static final double P = 10;
  public static final double I = 0;
  public static final double D = 0;

  public static final double S = 0;
  public static final double V = 1;
  public static final double G = 0;
  public static final double A = 0;

  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCELERATION = 20;
  public static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  public static final double GEARING = 5;
  public static final double MOI = 5;
  public static final double LENGTH = 5;
  public static final Angle MIN_ANGLE = Radians.of(0);
  public static final Angle MAX_ANGLE = Radians.of(Math.PI * 1 / 2);
  public static final Angle START_ANGLE = Radians.of(0);

  public static final Angle POSITION_TOLERANCE = Radians.of(0.05);
  public static final double[] MEASUREMENT_STDEVS = {0, 5};
}
