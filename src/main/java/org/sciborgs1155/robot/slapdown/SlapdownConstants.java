package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class SlapdownConstants {
  public static final Current CURRENT_LIMIT = Amps.of(5);

  public static final double ROLLER_VOLTAGE = 10;
  public static final double EXTEND_VOLTAGE = 10;

  public static final double kP = 10;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 1;
  public static final double kG = 0;
  public static final double kA = 0;

  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCELERATION = 20;

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  public static final double GEARING = 5;
  public static final double MOI = 5;
  public static final double LENGTH = 5;
  public static final Angle MIN_ANGLE = Radians.of(0);
  public static final Angle MAX_ANGLE = Radians.of(Math.PI * 1 / 2);
  public static final Angle START_ANGLE = Radians.of(0);

  public static final Angle POSITION_TOLERANCE = Radians.of(0.05);
  public static final double[] measurementStDevs = {0, 5};
}
