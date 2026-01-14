package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class TurretConstants {
  public static final CANBus CAN_BUS = new CANBus(); // TODO: Update.
  public static final Current CURRENT_LIMIT = Amps.of(60);

  public static final double GEAR_RATIO = 100.0;

  public static final double CONVERSION_FACTOR = (2.0 * Math.PI) / GEAR_RATIO;

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.026); // TODO: Update.

  public static final Distance TURRET_LENGTH = Meters.of(0.1);

  public static final Angle MAX_ANGLE = Radians.of(Math.PI);
  public static final Angle MIN_ANGLE = Radians.of(-Math.PI);

  public static final Angle START_ANGLE = Radians.of(0);
}
