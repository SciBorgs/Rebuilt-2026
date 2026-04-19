package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.EPS;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.hood.HoodConstants;

/** Constants used in the shooting algorithm. */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public final class ShootingConstants {
  // PREVENTS INSTANTIATION
  private ShootingConstants() {}

  public static final String TABLE_DIRECTORY =
      Robot.isReal() ? Filesystem.getDeployDirectory() + "/shooting/" : "resources/shooting/";

  public static final class DistanceTableConstants {
    /** Array indices for data stored within the DistanceTable. */
    public static final int DISTANCE = 0, ROLLER_SPEED = 1, HOOD_ANGLE = 2, TIME_OF_FLIGHT = 3;

    /** The path to the standard lookup table (within the resources folder). */
    public static final String TABLE_PATH = "DistanceTable";

    public static final String DELIMITER = ",";
  }

  public static final class RollerTableConstants {
    /** Array indices for data stored within the RollerTable. */
    public static final int LAUNCH_SPEED = 0, ROLLER_SPEED = 1, ERROR = 2;

    /** The path to the velocity lookup table (within the resources/shooting folder). */
    public static final String TABLE_PATH = "RollerTable";

    public static final String DELIMITER = ",";
    public static final String ELEMENT_FORMAT = "%.4f";

    /** [SPEED,ROLLER_SPEED,ERROR] */
    public static final String FORMAT =
        ELEMENT_FORMAT + DELIMITER + ELEMENT_FORMAT + DELIMITER + ELEMENT_FORMAT;
  }

  public static final class ParameterTableConstants {
    /** Array indices for data stored within the ParameterTable. */
    public static final int DISTANCE = 0, SPEED = 1, PITCH = 2, ERROR = 3;

    /** The resolution of the lookup table, in entries per meter. */
    public static final double DISTANCE_RESOLUTION = 100;

    public static final int MAX_LOOKUP_TABLE_SIZE = 50000;

    /** Lookup Table entries with errors greater than this are removed from the table. */
    public static final double MAX_ERROR = ScoringConstants.SCORE_RADIUS;

    /** The path to the parameter lookup table (within the resources/shooting folder). */
    public static final String PARAMETER_TABLE_PATH = "ParameterTable";

    public static final String DELIMITER = ",";
    public static final String ELEMENT_FORMAT = "%.4f";

    /** [DISTANCE,SPEED,PITCH,ERROR] */
    public static final String FORMAT =
        ELEMENT_FORMAT
            + DELIMITER
            + ELEMENT_FORMAT
            + DELIMITER
            + ELEMENT_FORMAT
            + DELIMITER
            + ELEMENT_FORMAT;
  }

  public static final class LaunchParameters {
    /** Array indices for storing launch parameters. */
    public static final int SPEED = 0, PITCH = 1, YAW = 2, ERROR = 2;
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
    /** Maximum allowable turret error for the indexer to run (% of full circle). */
    public static final double TURRET_ERROR_THRESHOLD = 10;

    /** Maximum allowable hood error for the indexer to run (% of full circle). */
    public static final double HOOD_ERROR_THRESHOLD = 10;

    /** Maximum allowable turret error for the indexer to run (% relative to setpoint). */
    public static final double SHOOTER_ERROR_THRESHOLD = 10;

    public static final double CLEARANCE = 0.5;
    public static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

    public static final double SCORE_DEPTH = 0;
    public static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

    /** Used in TOF analysis. */
    public static final double TOF_DEPTH = 0;

    /** Used in TOF analysis. */
    public static final double TOF_RADIUS = Hub.INNER_WIDTH / 2;

    /** The target translation for the FUEL to hit. */
    public static final double[] GOAL = fromTranslation(Hub.TOP_CENTER_POINT);
  }

  public static final class OptimizerConstants {
    public static final double TOF_KP = 0.5;
    public static final double TOF_KD = 0.05;

    public static final int MAX_TOF_ANALYSIS_ITERATIONS = 3000;
    public static final double TOF_ANALYSIS_THRESHOLD = 0.001;

    public static final double MAX_AIR_TIME = 10;

    public static final double SPEED_KP = 0.5;
    public static final double SPEED_KD = 0.05;

    public static final int MAX_OPTIMIZER_ITERATIONS = 3000;
    public static final double OPTIMIZATION_THRESHOLD = 0.01;

    /** The resolution of the trajectory used in the ShotOptimizer. */
    public static final int RESOLUTION = 1000;

    /** The resolution of the pitches in the lookup table, in samples per 2pi radians. */
    public static final double PITCH_RESOLUTION = 512;
  }

  public static final class PhysicalConstants {
    /** The translation from the center of the robot the center of the turret */
    public static final double[] ROBOT_TO_SHOOTER = {
      CENTER_TO_SHOOTER.getX(), CENTER_TO_SHOOTER.getY(), CENTER_TO_SHOOTER.getZ()
    };

    public static final double[] SHOOTER_TO_FLYWHEEL = {0.105803, 0, 0.061220};

    public static final boolean DRAG_ENABLED = true;
    public static final boolean LIFT_ENABLED = false;

    public static final double FUEL_MASS = 0.225;
    public static final double FUEL_RADIUS = 0.075;

    public static final double MAX_SPEED = 10;
    public static final double MIN_SPEED = EPS;

    public static final double MIN_PITCH = toPitch(HoodConstants.MAX_ANGLE.in(Radians));
    public static final double MAX_PITCH = toPitch(HoodConstants.MIN_ANGLE.in(Radians));

    public static final double MIN_DISTANCE = 2.5;
    public static final double MAX_DISTANCE = 5;

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

  public static final class TestingConstants {
    public static final double SPEED_TOLERANCE = 1; // meters/s
    public static final double ANGLE_TOLERANCE = 15; // degrees
    public static final double DISTANCE_TOLERANCE = 0.5; // meters

    public static final double AIRTIME_TEST_SPEED = 7.190; // meter/s
    public static final double AIRTIME_TEST_PITCH = 1.124; // rads
    public static final double AIRTIME_TEST_DISTANCE = 0.007; // meters

    public static final double ACCURACY_TEST_SPEED = 6.955; // meter/s

    public static final double SPEED_TEST_SPEED = 7.951; // meter/s
  }
}
