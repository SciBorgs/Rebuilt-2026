package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {

  public static final double P = 10;
  public static final double I = 0;
  public static final double D = 0;

  public static final double S = 0;
  public static final double G = 0;
  public static final double V = 0;
  public static final double A = 0;

  public static final double EXTRA_VOLT_FOR_SIM = 1;

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(.5);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(.2);

  public static final int LEFT_MOTOR_ID = 0;
  public static final int RIGHT_MOTOR_ID = 0;
  public static final double GEARING = 180.0;
  public static final Current STATOR_LIMIT = Amps.of(60);
  public static final Current SUPPLY_LIMIT = Amps.of(100);
  public static final Distance POSITION_TOLERANCE = Centimeters.of(3.5);

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
  public static final double MOI = 0.001;
  public static final Distance MIN_HEIGHT = Centimeters.of(10);
  public static final Distance MAX_HEIGHT = Centimeters.of(30);
  public static final Mass WEIGHT = Pounds.of(6.42);
  public static final Distance SPROCKET_RADIUS = Inches.of(5);

  public static final boolean TUNING = true;
}
