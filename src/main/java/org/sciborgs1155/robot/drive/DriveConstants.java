package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.List;

/**
 * Constants for our 2025 Swerve X2t drivetrain! All fields in this file should be updated for the
 * current robot configuration!
 */
public final class DriveConstants {
  // TODO: Change central drivetrain constants as needed.

  // The type of module on the chassis
  public static final ModuleType TYPE = ModuleType.TALON;

  // The control loop used by all of the modules when driving
  public static final ControlMode DRIVE_MODE = ControlMode.OPEN_LOOP_VELOCITY;

  // Rate at which sensors update periodicially
  public static final Time SENSOR_PERIOD = Seconds.of(0.02);

  // Distance between centers of right and left wheels on robot
  public static final Distance TRACK_WIDTH =
      Inches.of(
          22); // TODO may be different on complete chassis (right now we're using the test chassis)
  // Distance between front and back wheels on robot
  public static final Distance WHEEL_BASE = Inches.of(22);
  // The radius of any swerve wheel
  public static final Distance WHEEL_RADIUS = Inches.of(1.75);
  // Distance from the center to any wheel of the robot
  public static final Distance RADIUS = TRACK_WIDTH.div(2).times(Math.sqrt(2));
  // Coefficient of friction between the drive wheel and the carpet.
  public static final double WHEEL_COF = 1.0;
  // Robot width with bumpers
  public static final Distance CHASSIS_WIDTH = Inches.of(32.645);

  // Maximum achievable translational and rotation velocities and accelerations of the robot.
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(40);
  public static final LinearAcceleration MAX_SKID_ACCEL =
      MetersPerSecondPerSecond.of(38); // TODO: Tune
  public static final LinearAcceleration MAX_TILT_ACCEL =
      MetersPerSecondPerSecond.of(12); // TODO: Tune
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / RADIUS.in(Meters));
  public static final AngularAcceleration MAX_ANGULAR_ACCEL =
      RadiansPerSecondPerSecond.of(MAX_ACCEL.in(MetersPerSecondPerSecond) / RADIUS.in(Meters));

  // Arbitrary max rotational velocity for the driver to effectively control the robot
  public static final AngularVelocity TELEOP_ANGULAR_SPEED = RadiansPerSecond.of(2 * PI);

  public static final Translation2d[] MODULE_OFFSET = {
    new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)), // front left
    new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(-2)), // front right
    new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(2)), // rear left
    new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(-2)) // rear right
  };

  // angular offsets of the modules, since we use absolute encoders
  // ignored (used as 0) in simulation because the simulated robot doesn't have offsets
  public static final List<Rotation2d> ANGULAR_OFFSETS =
      List.of(
          Rotation2d.kZero, // front left
          Rotation2d.kZero, // front right
          Rotation2d.kZero, // rear left
          Rotation2d.kZero // rear right
          );

  public static final Rotation3d GYRO_OFFSET = new Rotation3d(0, 0, PI);

  /** The type of control loop to use when controlling a module's drive motor. */
  public enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  public record FFConstants(double kS, double kV, double kA) {}

  /** The type of modules being used. */
  public enum ModuleType {
    TALON, // Kraken X60 Drive, Kraken X60 Turn
    SPARK; // NEO Vortex Drive, NEO 550 Turn
  }

  public static final class Assisted {
    // The angle between the velocity and the displacement from a target, above which the robot will
    // not use assisted driving to the target. (the driver must be driving in the general direction
    // of the assisted driving target.)
    public static final Angle DRIVING_THRESHOLD = Radians.of(PI / 6);

    // The input of the joystick beyond which the assisted driving will not control the rotation of
    // the swerve.
    public static final double ROTATING_THRESHOLD = 0.02;
  }

  public static final class Skid {
    // TODO: find a value (3 is currently random, should change)
    public static final LinearVelocity THRESHOLD = MetersPerSecond.of(3);
  }

  // TODO: Change ALL characterization constants for each unique robot as needed.
  public static final class Translation {
    public static final double P = 4.0;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Distance TOLERANCE = Centimeters.of(1);
  }

  public static final class Rotation {
    public static final double P = 4.5;
    public static final double I = 0.0;
    public static final double D = 0.05;

    public static final Angle TOLERANCE = Degrees.of(2);
  }

  public static final class ModuleConstants {
    public static final double COUPLING_RATIO = 0;

    public static final class Driving {
      public static final Distance CIRCUMFERENCE = WHEEL_RADIUS.times(2 * PI);

      public static final double GEARING = 5.68;

      public static final Current STATOR_LIMIT = Amps.of(80); // 120A max slip current
      public static final Current SUPPLY_LIMIT = Amps.of(70);

      // TODO these factors are for SparkModule only!
      public static final Distance POSITION_FACTOR = CIRCUMFERENCE.times(GEARING);
      public static final LinearVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

      public static final Current CURRENT_LIMIT = Amps.of(50);

      public static final FFConstants FRONT_RIGHT_FF = new FFConstants(0.21459, 2.0025, 0.094773);
      public static final FFConstants FRONT_LEFT_FF = new FFConstants(0.23328, 2.0243, 0.045604);
      public static final FFConstants REAR_LEFT_FF = new FFConstants(0.14362, 2.0942, 0.21547);
      public static final FFConstants REAR_RIGHT_FF = new FFConstants(0.15099, 1.9379, 0.30998);

      public static final List<FFConstants> FF_CONSTANTS =
          List.of(FRONT_LEFT_FF, FRONT_RIGHT_FF, REAR_LEFT_FF, REAR_RIGHT_FF);

      public static final class PID {
        public static final double P = 3.2;
        public static final double I = 0.0;
        public static final double D = 0.0;
      }
    }

    public static final class Turning {
      public static final double GEARING = 12.1;
      public static final double ENCODER_GEARING = 1;

      public static final Current CURRENT_LIMIT = Amps.of(20);

      // TODO these factors are for SparkModule only!
      public static final Angle POSITION_FACTOR = Rotations.of(ENCODER_GEARING);
      public static final AngularVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

      public static final class PID {
        public static final double P = 50;
        public static final double I = 0.0;
        public static final double D = 0.05;
      }

      // system constants only used in simulation
      public static final class FF {
        public static final double S = 0.30817;
        public static final double V = 0.55;
        public static final double A = 0.03;
      }
    }
  }
}
