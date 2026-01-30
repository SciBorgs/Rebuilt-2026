package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/** Constants used in the {@code Turret} subsystem. */
public class TurretConstants {
  public static final CANBus CAN_BUS = new CANBus();
  public static final Current CURRENT_LIMIT = Amps.of(60);

  public static final double GEAR_RATIO = 100.0; // TODO: Update.
  public static final double SENSOR_TO_MECHANISM_RATIO = 2 * Math.PI / GEAR_RATIO; // TODO: Update.
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.026); // TODO: Update.
  public static final Constraints CONSTRAINTS = new Constraints(0, 0); // TODO: Update.
  public static final Distance TURRET_LENGTH = Meters.of(0.1); // TODO: Update.

  public static final Angle MAX_ANGLE = Degrees.of(175);
  public static final Angle MIN_ANGLE = Degrees.of(-175);
  public static final Angle START_ANGLE = Radians.of(0);

  public static final int VISUALIZER_WIDTH = 6;
  public static final int VISUALIZER_HEIGHT = 6;

  public static final class ControlConstants {
    // PID CONSTANTS
    public static final double P = 1;
    public static final double I = 0;
    public static final double D = 0;

    // FEEDFORWARD CONSTANTS (VELOCITY IN RAD/SEC)
    public static final double S = 0; // TODO: Update.
    public static final double V = 0; // TODO: Update.
    public static final double A = 0; // TODO: Update.

    // TOLERANCES
    public static final Angle TOLERANCE = Degree.of(20);
  }

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.2).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.5);
  public static final Time TIME_OUT = Seconds.of(6);
}
