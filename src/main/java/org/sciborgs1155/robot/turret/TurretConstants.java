package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Constants used in the {@code Turret} subsystem. */
public class TurretConstants {
  public static final CANBus CAN_BUS = new CANBus(); // TODO: Update.
  public static final Current CURRENT_LIMIT = Amps.of(60); // TODO: Update.

  public static final double GEAR_RATIO = 100.0; // TODO: Update.

  public static final double CONVERSION_FACTOR = 2.0 * Math.PI / GEAR_RATIO; // TODO: Update.

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.026); // TODO: Update.

  public static final Distance TURRET_LENGTH = Meters.of(0.1); // TODO: Update.

  public static final Angle MAX_ANGLE = Radians.of(Math.PI); // TODO: Update.
  public static final Angle MIN_ANGLE = Radians.of(-Math.PI); // TODO: Update.

  public static final Angle START_ANGLE = Radians.of(0); // TODO: Update.

  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(0); // TODO: Update.
  public static final AngularAcceleration MAX_ACCELERATION =
      RadiansPerSecondPerSecond.of(0); // TODO: Update.

  public static final class PID {
    public static final double P = 1; // TODO: Update.
    public static final double I = 0; // TODO: Update.
    public static final double D = 0; // TODO: Update.

    public static final Angle POSITION_TOLERANCE = Radians.of(0.01); // TODO: Update.
    public static final AngularVelocity VELOCITY_TOLERANCE =
        RadiansPerSecond.of(0.01); // TODO: Update.

    public static final Constraints CONSTRAINTS =
        new Constraints(
            MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond));
  }

  public static final class FF {
    public static final double S = 0; // TODO: Update.
    public static final double V = 0; // TODO: Update.
    public static final double A = 0; // TODO: Update.
  }
}
