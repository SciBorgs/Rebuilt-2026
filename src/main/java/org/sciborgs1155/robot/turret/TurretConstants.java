package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Current;

public class TurretConstants {
  public static final CANBus CAN_BUS = new CANBus(); // TODO: Update.
  public static final double CONVERSION_FACTOR = 1; // TODO: Update.

  public static final Current CURRENT_LIMIT = Amps.of(67); // TODO: Update.
}
