package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.EPS;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.turret.TurretConstants;

/** Constants used in the shooting algorithm. */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public final class ShootingConstants {
  public static final String DIRECTORY =
      Robot.isReal() ? Filesystem.getDeployDirectory() + "/shooting/" : "resources/shooting/";

  // PREVENTS INSTANTIATION
  private ShootingConstants() {}

  public static final class CalibrationConstants {
    public static final double HUB_BUFFER = 0.1;

    public static final double MIN_DISTANCE = Hub.WIDTH / 2 + DriveConstants.CHASSIS_WIDTH.in(Meters) / 2 + HUB_BUFFER;
    public static final double MAX_DISTANCE = 5;

    public static final int ENTRIES = 10;
    public static final double INCREMENT = (MAX_DISTANCE - MIN_DISTANCE) / ENTRIES;

    public static final double STARTING_ROLLER_SPEED = 200;

    /** Array indices for data stored within the DistanceTable. */
    public static final int DISTANCE = 0, ROLLER_SPEED = 1, HOOD_ANGLE = 2, TIME_OF_FLIGHT = 3;

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

    public static final double MAX_YAW = TurretConstants.MAX_ANGLE.in(Radians);
    public static final double MIN_YAW = TurretConstants.MIN_ANGLE.in(Radians);

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
}
