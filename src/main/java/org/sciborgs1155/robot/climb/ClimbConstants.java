package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static org.sciborgs1155.robot.climb.ClimbConstants.EXTRA_VOLT_FOR_SIM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {

  public static final double P = 1000;
  public static final double I = 0;
  public static final double D = 0;

  public static final double EXTRA_VOLT_FOR_SIM = 2;

  public static final double S = 0;
  public static final double G = EXTRA_VOLT_FOR_SIM;
  public static final double V = 0;
  public static final double A = 0;

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(.5);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(.2);

  public static final double GEARING = 60.0; // TODO
  public static final Current STATOR_LIMIT = Amps.of(60);
  public static final Current SUPPLY_LIMIT = Amps.of(100);
  public static final Distance POSITION_TOLERANCE = Inches.of(1.5);

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
  public static final double MOI = 0.001;
  public static final Distance MIN_HEIGHT = Inches.of(2.032);
  public static final Distance MAX_HEIGHT = Inches.of(30);
  public static final Distance STARTING_HEIGHT = MAX_HEIGHT;
  public static final Mass WEIGHT = Pounds.of(6.42); // TODO
  public static final Distance SPROCKET_RADIUS = Inches.of(5);
}
