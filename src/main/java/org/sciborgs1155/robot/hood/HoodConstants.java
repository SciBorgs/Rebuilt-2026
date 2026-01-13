package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {

  public static final double GEARING = 0;
  public static final double HOOD_LENGTH = 0;
  public static final double MAX_ANGLE = 0;
  public static final double MIN_ANGLE = 0;
  public static final double MOI = 0;
  public static final double STARTING_ANGLE = 0;
  public static final double CANCODER_GEARING = 0;
  public static final Current SUPPLY_LIMIT = Amps.of(0);
  public static final Current STATOR_LIMIT = Amps.of(0);
  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(1);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(1);

  public static final Angle POS_TOLERANCE = Radians.of(0.1);

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.5).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.3);
  public static final Time TIME_OUT = Seconds.of(3);

  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kG = 0;
  public static final double kA = 0;
}
