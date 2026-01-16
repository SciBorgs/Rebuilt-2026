package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
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

  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(.5);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(.2);

  public static final int MOTOR_ID = 0;
  public static final double GEARING = 50.0;
  public static final double SENSOR_TO_MECHANISM_RATIO = 2.0;
  public static final Current STATOR_LIMIT = Amps.of(60);
  public static final Current SUPPLY_LIMIT = Amps.of(100);
  public static final Distance POSITION_TOLERANCE = Centimeters.of(3.5);

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
  public static final double MOI = 0.001;
  public static final Distance MIN_HEIGHT = Centimeters.of(10);
  public static final Distance MAX_HEIGHT = Centimeters.of(30);
  public static final Mass WEIGHT = Pounds.of(6.42);
  public static final Distance SPROCKET_RADIUS = Inches.of(5);

  public enum Level {
    L1(Meters.of(0.3)),
    L2(Meters.of(0.488)),
    L3(Meters.of(0.838)),
    L4(Meters.of(1.408)),

    L2_ALGAE(Meters.of(0.119)),
    L3_ALGAE(Meters.of(0.496));

    public final Distance extension;

    Level(Distance extension) {
      this.extension = extension;
    }
  }
}
