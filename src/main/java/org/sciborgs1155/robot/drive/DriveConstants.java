package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.PI;

import com.ctre.phoenix6.signals.SensorDirectionValue;
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
 * Constants for our 2026 Swerve X2t drivetrain! All fields in this file should be updated for the
 * current robot configuration!
 */
public final class DriveConstants {
  // TODO: Change central drivetrain constants as needed.

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

  // Percentage of the max speed we can do while shooting
  public static final double SHOOTING_TRANSLATIONAL_SPEED = 0.5;
  public static final double SHOOTING_ANGULAR_SPEED = 0.5;

  // Maximum achievable translational and rotation velocities and accelerations of the robot.
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(40);
  public static final LinearAcceleration MAX_SKID_ACCEL =
      MetersPerSecondPerSecond.of(28); // TODO: Tune
  public static final LinearAcceleration MAX_TILT_ACCEL =
      MetersPerSecondPerSecond.of(20); // TODO: Tune
  // Percentage of the max angular speed we can do while shooting
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
          Rotation2d.fromRotations(0.093505859375), // front left
          Rotation2d.fromRotations(0.076416015625), // front right
          Rotation2d.fromRotations(0.371826171875), // rear left
          Rotation2d.fromRotations(-0.280517578125) // rear right
          );

  public static final Rotation3d GYRO_OFFSET = new Rotation3d(0, 0, PI);

  /** The type of control loop to use when controlling a module's drive motor. */
  public enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  public record FFConstants(double kS, double kV, double kA) {}

  public record PIDConstants(double kP, double kI, double kD) {}

  public static final class Assisted {
    // The angle between the velocity and the displacement from a target, above which the robot will
    // not use assisted driving to the target. (the driver must be driving in the general direction
    // of the assisted driving target.)
    public static final Angle DRIVING_THRESHOLD = Radians.of(PI / 6);

    // The input of the joystick beyond which the assisted driving will not control the rotation of
    // the swerve.
    public static final double ROTATING_THRESHOLD = 0.02;
  }

  // public static final class Skid {
  //   // TODO: find a value (3 is currently random, should change)
  //   public static final LinearVelocity THRESHOLD = MetersPerSecond.of(3);
  // }

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

    public static final class Driving {
      public static final Distance CIRCUMFERENCE = WHEEL_RADIUS.times(2 * PI);

      public static final double GEARING = 5.68;

      public static final Current STATOR_LIMIT = Amps.of(80); // 120A max slip current
      public static final Current SUPPLY_LIMIT = Amps.of(70);

      public static final Current CURRENT_LIMIT = Amps.of(80);

      public static final FFConstants FRONT_RIGHT_FF = new FFConstants(0.18984, 2.5193, 0.22823);
      public static final FFConstants FRONT_LEFT_FF = new FFConstants(0.19491, 2.2614, 0.041899);
      public static final FFConstants REAR_LEFT_FF = new FFConstants(0.08037, 2.2685, 0.22776);
      public static final FFConstants REAR_RIGHT_FF = new FFConstants(0.20194, 2.5175, 0.19175);

      public static final List<FFConstants> FF_CONSTANTS =
          List.of(FRONT_LEFT_FF, FRONT_RIGHT_FF, REAR_LEFT_FF, REAR_RIGHT_FF);

      public static final class PID {
        public static final double P = 1;
        public static final double I = 0.0;
        public static final double D = 0;
      }
    }

    public static final class Turning {
      public static final double GEARING = 12.1;

      // Must match the turn motor's positive direction at the module output shaft.
      // Verify on bench: rotate module clockwise — if encoder reads negative, flip to Clockwise_Positive.
      public static final SensorDirectionValue ENCODER_DIRECTION =
          SensorDirectionValue.CounterClockwise_Positive;

      public static final Current CURRENT_LIMIT = Amps.of(20);

      // MotionMagicExpo profile gains — tune with SysId rotationalCharacterization.
      public static final double EXPO_KV = 0.12; // V per rps of module
      public static final double EXPO_KA = 0.10; // V per rps^2 of module

      public static final PIDConstants FRONT_RIGHT_PID = new PIDConstants(60, 0,0.5);
      public static final PIDConstants REAR_LEFT_PID = new PIDConstants(60, 0, 0.5);
      public static final PIDConstants REAR_RIGHT_PID = new PIDConstants(60, 0, 0.5);
      public static final PIDConstants FRONT_LEFT_PID = new PIDConstants(60, 0, 0.5);

      public static final List<PIDConstants> PID_CONSTANTS =
          List.of(FRONT_LEFT_PID, FRONT_RIGHT_PID, REAR_LEFT_PID, REAR_RIGHT_PID);

      public static final FFConstants FRONT_RIGHT_FF = new FFConstants(0.0, 0.0, 0);
      public static final FFConstants FRONT_LEFT_FF = new FFConstants(0.0, 0.0, 0);
      public static final FFConstants REAR_LEFT_FF = new FFConstants(0.0, 0.0, 0);
      public static final FFConstants REAR_RIGHT_FF = new FFConstants(0.0, 0.0, 0);

      public static final List<FFConstants> FF_CONSTANTS =
          List.of(FRONT_LEFT_FF, FRONT_RIGHT_FF, REAR_LEFT_FF, REAR_RIGHT_FF);
    }

    public static final class SIM_FF {
      public static final double S = 0.30817;
      public static final double V = 0.55;
      public static final double A = 0.03;
    }
  }
}
