package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
public class Constants {
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
  public static enum RobotType {
    FULL,
    CHASSIS,
    NONE
  }

  /** The current robot state, as in the type. Remember to update! */
  public static RobotType ROBOT_TYPE = RobotType.FULL;

  /** States if we are in tuning mode. Ideally, keep it at false when not used. */
  public static boolean TUNING = false;

  // TODO: UPDATE ALL OF THESE VALUES.
  /** Describes physical properites of the robot. */
  public static class Robot {
    public static final Mass MASS = Kilograms.of(25);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.2);
  }

  public static final Time PERIOD = Seconds.of(0.02); // roborio tickrate (s)
  public static final Time ODOMETRY_PERIOD = Seconds.of(1.0 / 20.0); // 4 ms (speedy!)
  public static final double DEADBAND = 0.15;
  public static final double MAX_RATE =
      DriveConstants.MAX_ACCEL.baseUnitMagnitude()
          / DriveConstants.MAX_ANGULAR_SPEED.baseUnitMagnitude();
  public static final double SLOW_SPEED_MULTIPLIER = 0.33;
  public static final double FULL_SPEED_MULTIPLIER = 1.0;

  // The name of seperate canivore, set to rio if no seperate canivore
  public static final String DRIVE_CANIVORE = "drivetrain";
}
