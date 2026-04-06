package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.Constants.EPS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_SPEED;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.sciborgs1155.lib.PolynomialRegression.ModelSelector.RegressionModel;
import org.sciborgs1155.lib.ProjectileVisualizer;
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

  private static boolean diff(double a, double b) {
    return Math.abs(a - b) > EPS;
  }

  // PREVENTS INSTANTIATION
  private ShootingConstants() {}

  public static final class ParameterLookupConstants {
    /** Array indices for data stored within the parameter table. */
    public static final int DISTANCE = 0, SPEED = 1, PITCH = 2;

    /** Identifiers for specific lookup models. */
    public enum LookupID {
      MINIMAL_AIR_TIME,
      MAXIMUM_ACCURACY
    }

    /** A map for each launch-parameter model. */
    public static final Map<LookupID, LaunchParameterLookup> parameterLookupSelector =
        new ConcurrentHashMap<>();

    static {
      ParameterLookup.addLookup(
          LookupID.MINIMAL_AIR_TIME,
          LaunchParameterLookup.fromCoefficients(
              new double[] {1.765, -0.440, 0.074, -0.005, 0.012},
              new double[] {6.807, -1.586, 0.962, -0.176, 0.012}));
    }
  }

  public static final class CalibrationConstants {
    public static final int ENTRIES = 10;
    public static final double INCREMENT = (MAX_DISTANCE - MIN_DISTANCE) / ENTRIES;

    public static final double STARTING_ROLLER_SPEED = 200;

    /** Array indices for data stored within the calibration table. */
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
    public static final double CLEARANCE = 0.2;
    public static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

    public static final double SCORE_DEPTH = 0;
    public static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

    /** Used in TOF analysis. */
    public static final double TOF_DEPTH = 0;

    /** Used in TOF analysis. */
    public static final double TOF_RADIUS = Hub.INNER_WIDTH / 2;

    /** The target translation for the FUEL to hit. */
    public static final double[] GOAL = ProjectileVisualizer.fromTranslation(Hub.TOP_CENTER_POINT);
  }

  public static final class OptimizerConstants {
    public static final double TOF_KP = 0.5;
    public static final double TOF_KD = 0.05;

    public static final int MAX_TOF_ANALYSIS_ITERATIONS = 3000;
    public static final double TOF_ANALYSIS_THRESHOLD = 0.001;

    public static final double MAX_AIR_TIME = 10;

    public static final double SPEED_KP = 0.5;
    public static final double SPEED_KD = 0.05;

    public static final double PITCH_KP = 0.5;
    public static final double PITCH_KD = 0.05;

    public static final int MAX_OPTIMIZER_ITERATIONS = 3000;
    public static final double OPTIMIZATION_THRESHOLD = 0.01;

    /** The resolution of the trajectory used in the ShotOptimizer. */
    public static final int RESOLUTION = 1000;

    /** The resolution of the pitches in the lookup table, in samples per 2pi radians. */
    public static final double PITCH_RESOLUTION = 720;
  }

  public static final class PhysicalConstants {
    public static final double HUB_BUFFER = 0.5;

    /** The translation from the center of the robot the center of the turret */
    public static final double[] ROBOT_TO_SHOOTER = {
      CENTER_TO_SHOOTER.getX(), CENTER_TO_SHOOTER.getY(), CENTER_TO_SHOOTER.getZ()
    };

    public static final double[] SHOOTER_TO_FLYWHEEL = {0.105803, 0, 0.061220};

    public static final boolean DRAG_ENABLED = true;
    public static final boolean LIFT_ENABLED = false;

    public static final double FUEL_MASS = 0.225;
    public static final double FUEL_RADIUS = 0.075;

    public static final double MAX_SPEED = 15;
    public static final double MIN_SPEED = EPS;

    public static final double MIN_PITCH = toPitch(HoodConstants.MAX_ANGLE.in(Radians));
    public static final double MAX_PITCH = toPitch(HoodConstants.MIN_ANGLE.in(Radians));

    public static final double MAX_YAW = TurretConstants.MAX_ANGLE.in(Radians);
    public static final double MIN_YAW = TurretConstants.MIN_ANGLE.in(Radians);

    public static final double MIN_DISTANCE =
        Hub.WIDTH / 2 + DriveConstants.CHASSIS_WIDTH.in(Meters) / 2 + HUB_BUFFER;
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

  /** Launch parameters for a single direct shot to the HUB (no yaw). */
  public static class DirectLaunchParameters {
    private double distance;
    private double speed;
    private double pitch;

    /** Launch parameters for a single direct shot to the HUB (no yaw). */
    public DirectLaunchParameters(double distance, double speed, double pitch) {
      this.distance = distance;
      this.speed = speed;
      this.pitch = pitch;
    }

    /** The planar distance of the shooter from the HUB in meters. */
    public double distance() {
      return distance;
    }

    /** The launch speed of the FUEL in meters per second. */
    public double speed() {
      return speed;
    }

    /** The launch pitch of the FUEL in radians. */
    public double pitch() {
      return pitch;
    }

    /**
     * Updates the launch distance parameter.
     *
     * @param distance the planar distance of the shooter from the HUB in meters
     */
    public void setDistance(double distance) {
      this.distance = distance;
    }

    /**
     * Updates the launch speed of the FUEL.
     *
     * @param speed the launch speed of the FUEL in meters per second
     */
    public void setSpeed(double speed) {
      this.speed = speed;
    }

    /**
     * Updates the launch pitch of the FUEL.
     *
     * @param pitch the launch pitch of the FUEL in radians
     */
    public void setPitch(double pitch) {
      this.pitch = pitch;
    }

    /**
     * Whether or not these launch parameters are the same as the given launch parameters.
     *
     * @param launchParameters the launch parameters to compare to
     */
    public boolean differsFrom(DirectLaunchParameters launchParameters) {
      return diff(distance, launchParameters.distance())
          || diff(speed, launchParameters.speed())
          || diff(pitch, launchParameters.pitch());
    }

    /** Whether or not the shot specified by this object is impossible. */
    public boolean isOutOfBounds() {
      return distance < MIN_DISTANCE
          || distance > MAX_DISTANCE
          || speed < MIN_SPEED
          || speed > MAX_SPEED
          || pitch < MIN_PITCH
          || pitch > MAX_PITCH;
    }
  }

  /**
   * Compiles polynomial regression models into a singular launch parameter model which acts as a
   * lookup table.
   */
  public static class LaunchParameterLookup {
    private final RegressionModel speedRegression;
    private final RegressionModel pitchRegression;

    /**
     * Compiles polynomial regression models into a singular launch parameter model which acts as a
     * lookup table.
     *
     * @param speedRegression the regression model for launch speed
     * @param pitchRegression the regression model for launch pitch
     */
    public LaunchParameterLookup(RegressionModel speedRegression, RegressionModel pitchRegression) {
      this.speedRegression = speedRegression;
      this.pitchRegression = pitchRegression;
    }

    /**
     * The launch speed of the FUEL from the given distance estimated using the model (meters per
     * second).
     *
     * @param distance the planar distance from the HUB to the shooter's origin, in meters
     */
    public double speed(double distance) {
      return speedRegression.predict(distance);
    }

    /** The coefficients for the speed regression model. */
    public double[] speedRegression() {
      return speedRegression.coefficients();
    }

    /**
     * The launch pitch of the FUEL from the given distance estimated using the model (radians).
     *
     * @param distance the planar distance from the HUB to the shooter's origin, in meters
     */
    public double pitch(double distance) {
      return pitchRegression.predict(distance);
    }

    /** The coefficients for the pitch regression model. */
    public double[] pitchRegression() {
      return pitchRegression.coefficients();
    }

    /**
     * Generates a new launch-parameter lookup from coefficient arrays.
     *
     * @param speedCoefficients the coefficients for the speed polynomial
     * @param pitchCoefficients the coefficients for the pitch polynomial
     */
    public static LaunchParameterLookup fromCoefficients(
        double[] speedCoefficients, double[] pitchCoefficients) {
      return new LaunchParameterLookup(
          new RegressionModel(speedCoefficients.length - 1, speedCoefficients),
          new RegressionModel(pitchCoefficients.length - 1, pitchCoefficients));
    }
  }
}
