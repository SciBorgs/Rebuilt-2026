package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class IntakeConstants {
  /**
   * We still want this for later, just add on more later TODO
   *
   * <p>You probably what the angular to linear scaling factors in here based on the dimensions
   * later
   */
  public static final Current CURRENT_LIMIT = Amps.of(5);

  public static final double ROLLER_VOLTAGE = 10;
  public static final double EXTEND_VOLTAGE = 10;

  public static final double kP = 0.01;
  public static final double kI = 0.01;
  public static final double kD = 0.01;

  public static final double kS = 0.01;
  public static final double kV = 0.01;
  public static final double kG = 0.01;
  public static final double kA = 0.01;

  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCELERATION = 20;

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(0);
  public static final double GEARING = 0;
  public static final double MOI = 0;
  public static final double LENGTH = 0;
  public static final Angle MIN_ANGLE = Radians.of(0);
  public static final Angle MAX_ANGLE = Radians.of(Math.PI * 1 / 2);
  public static final double START_ANGLE = 0;

  public static final Angle POSITION_TOLERANCE = Radians.of(5);
}
