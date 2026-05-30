package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.drive.DriveConstants;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @see Units
 */
public final class Constants {
  public static final Time PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final Time ODOMETRY_PERIOD = Seconds.of(1.0 / 100.0); // 4 ms (speedy!)
  public static final double DEADBAND = 0.2;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 1.0;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;

  // The name of seperate canivore, set to rio if no seperate canivore
  public static final CANBus DRIVE_CANIVORE = new CANBus("");
  public static final CANBus INTAKE_CANIVORE = new CANBus("intake");
  public static final CANBus SHOOTING_CANIVORE = new CANBus("shooting");

  /** The current robot state, as in the type. Remember to update! */
  public static final RobotType ROBOT_TYPE = RobotType.FULL;

  /** States if we are in tuning mode. Ideally, keep it at false when not used. */
  public static final boolean TUNING = true;

  public static final Pose2d CLIMB_POSE = new Pose2d(1.52, 3.74, new Rotation2d(Degrees.of(180)));

  // Prevents instantiation
  private Constants() {}

  // TODO: Modify as needed.
  /** Returns the robot's alliance. */
  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** Returns the rotation of the robot's alliance with respect to the origin. */
  public static Rotation2d allianceRotation() {
    return Rotation2d.fromRotations(alliance() == Alliance.Blue ? 0 : 0.5);
  }

  /** Defines the various types the robot can be. Useful for only using select subsystems. */
  public enum RobotType {
    FULL,
    CHASSIS,
    NONE
  }

  /** Lookup tables mapping shot distance (meters) to shooter parameters. */
  public static final class ShootingData {
    public static final DoubleEntry SIGGYS_CONSTANT =
        Tuning.entry("Robot/shooting/siggysConstant", 0.0);
      

    // minimum velocity to use SOTM algorithm rather than stationary. measured in m/s
    public static final double MINIMUM_VELOCITY = 0.01;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_RADS =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TOF =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_HORIZONTAL_VELOCITY =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap VELOCITY_TO_RADS =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingTreeMap<Double, Rotation2d> VELOCITY_TO_HOOD_ANGLE =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    private ShootingData() {
      
    }

    public static Command changeSC(double delta){
      return Commands.runOnce(() -> SIGGYS_CONSTANT.set(SIGGYS_CONSTANT.get()+delta));
    }

    /**
     * Applies a point to the three linear interpolations.
     *
     * @param dist The input distance.
     * @param degIncline The output degree of incline.
     * @param speed The
     * @param tof
     */
    public static void put(double dist, double degIncline, double speed, double tof) {
      Rotation2d hoodAngle = Rotation2d.fromDegrees(degIncline);
      DISTANCE_TO_HOOD_ANGLE.put(dist, hoodAngle);
      DISTANCE_TO_RADS.put(dist, speed);
      DISTANCE_TO_TOF.put(dist, tof);

      double velocity = dist / tof;
      DISTANCE_TO_HORIZONTAL_VELOCITY.put(dist, velocity);
      VELOCITY_TO_RADS.put(velocity, speed);
      VELOCITY_TO_HOOD_ANGLE.put(velocity, hoodAngle);
    }

    static {
      put(2.460, 15, 140, 0.98);
      put(2.555, 15, 147, 1.0);
      put(3.022, 26, 135, 1.0);
      put(3.309, 28, 138, 0.98);
      put(4.079, 30, 147, 1.34);
      // put(4.766, 34, 166, 1.06);

      // put(2.33, 26, 166.7, 0.96);
      // put(3.4, 30, 195, 1.05);

      // put(1.37, 15, 135, 0.89);
      // put(1.873, 18, 148, 1.08);
      // put(2.518, 27, 130, 0.95);
      // put(3.605, 30, 153, 1.09);
      // put(4.58, 34, 173, 1.09);
      // put(5.67, 38, 195, 1.25);
    }
  }

  // TODO: UPDATE ALL OF THESE VALUES.
  /** Describes physical properites of the robot. */
  public static class Robot {
    public static final Mass MASS = Pounds.of(130.37);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(6.9);
    public static final Distance SHOOTER_LENGTH = Inches.of(4); // TODO: UPDATE.
    public static final Translation3d ROBOT_TO_SHOOTER =
        new Translation3d(-0.14006, 0.13983, 0.3286252);
  }


}
