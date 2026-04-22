package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.Constants.EPS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.sciborgs1155.lib.ProjectileVisualizer;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.commands.shooting.ParameterLookup.LaunchParameterLookup;
import org.sciborgs1155.robot.commands.shooting.VelocityLookup.CalibrationEntry;
import org.sciborgs1155.robot.hood.HoodConstants;

/** Constants used in the shooting algorithm. */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public final class ShootingConstants {
  protected static boolean diff(double a, double b) {
    return Math.abs(a - b) > EPS;
  }

  // PREVENTS INSTANTIATION
  private ShootingConstants() {}

  public static final class ParameterLookupConstants {
    public static final int PARAMETER_ENTRIES = 50;
    public static final double PARAMETER_INCREMENT =
        (MAX_DISTANCE - MIN_DISTANCE) / PARAMETER_ENTRIES;

    /** Array indices for data stored within the parameter table. */
    public static final int DISTANCE = 0, SPEED = 1, PITCH = 2, YAW = 3;

    public static final int DEGREE = 3;

    /** Identifiers for specific lookup models. */
    public enum LookupID {
      MINIMAL_AIR_TIME,
      MAXIMUM_SPEED
    }

    /** A map for each launch-parameter lookup model. */
    public static final Map<LookupID, LaunchParameterLookup> parameterLookupSelector =
        new ConcurrentHashMap<>();

    static {
      ParameterLookup.addLookup(
          LookupID.MINIMAL_AIR_TIME,
          new LaunchParameterLookup(
              new double[] {5.396, 0.189, 0.150, -0.011},
              new double[] {1.844, -0.491, 0.079, -0.005},
              MIN_DISTANCE,
              MAX_DISTANCE));

      ParameterLookup.addLookup(
          LookupID.MAXIMUM_SPEED,
          new LaunchParameterLookup(
              new double[] {3.797, 1.939, -0.127, 0.008},
              new double[] {1.309, 0.0, 0.0, 0.0},
              MIN_DISTANCE,
              MAX_DISTANCE));

      VelocityLookup.useData(
          new CalibrationEntry[] {
            new CalibrationEntry(1.37, 135, Units.degreesToRadians(15)),
            new CalibrationEntry(1.87, 148, Units.degreesToRadians(18)),
            new CalibrationEntry(2.52, 130, Units.degreesToRadians(27)),
            new CalibrationEntry(3.61, 153, Units.degreesToRadians(30)),
            new CalibrationEntry(4.58, 173, Units.degreesToRadians(34)),
            new CalibrationEntry(5.67, 195, Units.degreesToRadians(38)),
          });
    }
  }

  public static final class CalibrationConstants {
    public static final int CALIBRATION_ENTRIES = 10;
    public static final double CALIBRATION_INCREMENT =
        (MAX_DISTANCE - MIN_DISTANCE) / CALIBRATION_ENTRIES;

    /** Array indices for data stored within the calibration table. */
    public static final int DISTANCE = 0, ROLLER_SPEED = 1, HOOD_ANGLE = 2;

    /** The path to the standard lookup table (within the resources folder). */
    public static final String TABLE_PATH = "CalibrationTable";

    public static final String DELIMITER = ",";
  }

  public static final class VisualizerConstants {
    /** The amount of FUEL able to be launched per second. */
    public static final int SHOOTING_SPEED = 5;

    /** The resolution of the visualizer's launch simulation. */
    public static final int VISUALIZER_RESOLUTION = 500;

    public static final double MAX_AIR_TIME = 10;

    public static final boolean TRAJECTORY_ENABLED = false;
    public static final boolean LAUNCH_ENABLED = true;
  }

  public static final class ScoringConstants {
    public static final double CLEARANCE = 0.2;
    public static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

    public static final double SCORE_DEPTH = 0;
    public static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

    public static final double CALIBRATION_DEPTH = 0;
    public static final double CALIBRATION_RADIUS = Hub.INNER_WIDTH / 2;

    /** The target translation for the FUEL to hit. */
    public static final double[] GOAL = ProjectileVisualizer.fromTranslation(Hub.TOP_CENTER_POINT);
  }

  public static final class OptimizerConstants {
    public static final double TOF_KP = 0.5;
    public static final double TOF_KD = 0.05;

    public static final int MAX_TOF_ANALYSIS_ITERATIONS = 300;
    public static final double TOF_ANALYSIS_THRESHOLD = 0.01;

    public static final double MAX_AIR_TIME = 10;

    public static final double SPEED_KP = 0.5;
    public static final double SPEED_KD = 0.05;

    public static final double PITCH_KP = 0.5;
    public static final double PITCH_KD = 0.05;

    public static final int MAX_OPTIMIZER_ITERATIONS = 300;
    public static final double OPTIMIZATION_THRESHOLD = 0.1;

    /** The resolution of the trajectory used in the ShotOptimizer. */
    public static final int RESOLUTION = 500;

    /** The resolution of the pitches in the lookup table, in samples per range. */
    public static final double PITCH_RESOLUTION = 500;

    /** The resolution of the speed in the lookup table, in samples per range. */
    public static final double SPEED_RESOLUTION = 100;
  }

  public static final class PhysicalConstants {
    public static final double HUB_BUFFER = 0.1;

    /** The translation from the center of the robot the center of the turret */
    public static final double[] ROBOT_TO_SHOOTER = {
      CENTER_TO_SHOOTER.getX(), CENTER_TO_SHOOTER.getY(), CENTER_TO_SHOOTER.getZ()
    };

    public static final double[] SHOOTER_TO_FLYWHEEL = {0.105803, 0, 0.061220};

    public static final boolean DRAG_ENABLED = true;
    public static final boolean LIFT_ENABLED = false;

    public static final double FUEL_MASS = 0.225;
    public static final double FUEL_RADIUS = 0.075;

    public static final double MIN_ROLLER_SPEED = 0;
    public static final double MAX_ROLLER_SPEED = 400;

    public static final double MAX_SPEED = 20;
    public static final double MIN_SPEED = 0;

    public static final double MIN_PITCH = toPitch(HoodConstants.MAX_ANGLE.in(Radians));
    public static final double MAX_PITCH = toPitch(HoodConstants.MIN_ANGLE.in(Radians));

    public static final double MAX_YAW = Integer.MAX_VALUE;
    public static final double MIN_YAW = Integer.MIN_VALUE;

    public static final double MIN_DISTANCE = 1.36;
    public static final double MAX_DISTANCE = 6.12;

    /**
     * The radius of the arc formed by the starting translation of the FUEL as the hood angle
     * changes.
     */
    public static final double SHOOTER_RADIUS = Units.inchesToMeters(2) + FUEL_RADIUS / 2;

    /** The distance above the Hub from which the FUEL's fate is decided (score / miss). */
    public static final double SCORE_WINDOW = FUEL_RADIUS / 2;

    /** Multiplied by velocity squared to compute drag acceleration. */
    public static final double DRAG_CONSTANT =
        0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS / FUEL_MASS;

    /** Multiplied by spin and velocity to produce magnus acceleration. */
    public static final double LIFT_CONSTANT =
        4.0 / 3.0 * Math.PI * FUEL_RADIUS * FUEL_RADIUS * FUEL_RADIUS * AIR_DENSITY / FUEL_MASS;
  }

  /**
   * Converts launch pitch to hood angle.
   *
   * @param pitch the pitch to launch the FUEL at in radians
   * @return the hood angle (in radians) such that the FUEL is launched at the specified pitch
   */
  public static double toHoodAngle(double pitch) {
    return Math.PI / 2 - pitch;
  }

  /**
   * Converts hood angle to launch pitch.
   *
   * @param hoodAngle the hood angle (in radians)
   * @return the pitch (in radians) such that the Hood will be at the specified angle
   */
  public static double toPitch(double hoodAngle) {
    return Math.PI / 2 - hoodAngle;
  }
}
