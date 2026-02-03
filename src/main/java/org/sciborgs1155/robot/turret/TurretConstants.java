package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/** Constants used in the {@code Turret} subsystem. */
public class TurretConstants {
  public static final Current CURRENT_LIMIT = Amps.of(60);

  public static final double GEAR_RATIO = 686 / 15; // TODO: Update.
  public static final double SENSOR_TO_MECHANISM_RATIO = 2 * Math.PI / GEAR_RATIO;
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0872);
  public static final Constraints CONSTRAINTS = new Constraints(16, 16); // TODO: Update.

  public static final Angle MAX_ANGLE = Degrees.of(175);
  public static final Angle MIN_ANGLE = Degrees.of(-175);
  public static final Angle START_ANGLE = Radians.of(0);

  public static final int VISUALIZER_WIDTH = 6;
  public static final int VISUALIZER_HEIGHT = 6;

  public static final class ControlConstants {
    // PID CONSTANTS
    public static final double P = 6.7;
    public static final double I = 0;
    public static final double D = 0;

    // FEEDFORWARD CONSTANTS (VELOCITY IN RAD/SEC)
    public static final double S = 0; // TODO: Update.
    public static final double V = 0; // TODO: Update.
    public static final double A = 0; // TODO: Update.

    // TOLERANCES
    public static final Angle TOLERANCE = Degree.of(3);
  }

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(2);
  public static final Time TIME_OUT = Seconds.of(6);

  public static final int TURRET_GEARING = 84;
  public static final int ENCODER_A_GEARING = 12;
  public static final int ENCODER_B_GEARING = 13;

  public static final Angle CRT_MATCH_TOLERANCE = Degrees.of(1);
}
