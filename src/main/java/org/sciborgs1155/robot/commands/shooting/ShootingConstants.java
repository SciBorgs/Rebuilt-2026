package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.lib.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.Constants.EPS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.HOOD_ANGLE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.ROLLER_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_PITCH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CALIBRATION_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CALIBRATION_RADIUS;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.sciborgs1155.lib.PolynomialRegression;
import org.sciborgs1155.lib.PolynomialRegression.ModelSelector;
import org.sciborgs1155.lib.ProjectileVisualizer;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.shooter.ShooterConstants;

/** Constants used in the shooting algorithm. */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public final class ShootingConstants {
  private static boolean diff(double a, double b) {
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

    /** Identifiers for specific lookup models. */
    public enum LookupID {
      MINIMAL_AIR_TIME,
      MAXIMUM_SPEED
    }

    /** A model for the conversion between launch speed and roller speed. */
    public static final VelocityLookup velocityLookup =
        new VelocityLookup(
            new double[][] {
              {1.512, 10, 0.262}, // ENTRY 1
              {1.972, 20, 0.262}, // ENTRY 2
              {2.433, 50, 0.262}, // ENTRY 3
              {2.894, 100, 0.262}, // ENTRY 4
              {3.355, 150, 0.262}, // ENTRY 5
              {3.816, 200, 0.262}, // ENTRY 6
              {4.277, 250, 0.262}, // ENTRY 7
              {4.277, 300, 0.262}, // ENTRY 8
              {5.198, 350, 0.262}, // ENTRY 9
              {5.658, 400, 0.262}, // ENTRY 10
            },
            ShooterConstants.MAX_VELOCITY.in(RadiansPerSecond));

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
    }
  }

  public static final class CalibrationConstants {
    public static final int CALIBRATION_ENTRIES = 10;
    public static final double CALIBRATION_INCREMENT =
        (MAX_DISTANCE - MIN_DISTANCE) / CALIBRATION_ENTRIES;

    public static final double STARTING_ROLLER_SPEED = 200;

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

    public static final double MAX_SPEED = 20;
    public static final double MIN_SPEED = 0;

    public static final double MIN_PITCH = toPitch(HoodConstants.MAX_ANGLE.in(Radians));
    public static final double MAX_PITCH = toPitch(HoodConstants.MIN_ANGLE.in(Radians));

    public static final double MAX_YAW = Integer.MAX_VALUE;
    public static final double MIN_YAW = Integer.MIN_VALUE;

    public static final double MIN_DISTANCE =
        Hub.WIDTH / 2 + DriveConstants.CHASSIS_WIDTH.in(Meters) / 2 + HUB_BUFFER;
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

  /** Combines polynomial regression models into a singular parameter lookup. */
  public static class LaunchParameterLookup {
    private final double minDistance, maxDistance;
    private final double[] speedCoefficients;
    private final double[] pitchCoefficients;

    /**
     * Combines polynomial regression models into a singular parameter lookup.
     *
     * @param speedRegression the polynomial coefficients to model launch speed as a function of
     *     distance
     * @param pitchRegression the polynomial coefficients to model launch pitch as a function of
     *     distance
     * @param minDistance the minimum distance from the HUB, in meters
     * @param maxDistance the maximum distance from the HUB, in meters
     */
    public LaunchParameterLookup(
        double[] speedCoefficients,
        double[] pitchCoefficients,
        double minDistance,
        double maxDistance) {
      this.speedCoefficients = speedCoefficients.clone();
      this.pitchCoefficients = pitchCoefficients.clone();

      this.minDistance = minDistance;
      this.maxDistance = maxDistance;
    }

    /**
     * The launch speed of the FUEL (meters per second).
     *
     * @param distance the planar distance from the HUB to the shooter's origin, in meters
     */
    public double speed(double distance) {
      return inBounds(distance) ? evaluate(distance, speedCoefficients) : 0;
    }

    /** The coefficients for the speed regression model. */
    public double[] speedCoefficients() {
      return speedCoefficients.clone();
    }

    /**
     * The launch pitch of the FUEL (radians).
     *
     * @param distance the planar distance from the HUB to the shooter's origin, in meters
     */
    public double pitch(double distance) {
      return inBounds(distance) ? evaluate(distance, pitchCoefficients) : 0;
    }

    /** The coefficients for the pitch regression model. */
    public double[] pitchCoefficients() {
      return pitchCoefficients.clone();
    }

    /**
     * Whether or not the lookup table covers the specified distance.
     *
     * @param distance the distance from the HUB, in meters
     */
    public boolean inBounds(double distance) {
      return distance >= minDistance && distance <= maxDistance;
    }

    /**
     * Evaluates the value of a polynomial at a specified x value.
     *
     * @param x the x value to input into the polynomial
     * @param coefficients an array of coefficients describing the polynomial
     */
    public static double evaluate(double x, double[] coefficients) {
      double result = 0;
      for (int degree = 0; degree < coefficients.length; degree++)
        result += coefficients[degree] * Math.pow(x, degree);
      return result;
    }
  }

  /**
   * Consolidates polynomial regression models to both convert roller speed to launch speed and
   * vice-versa.
   */
  public static class VelocityLookup {
    /** The polynomial coefficients to model roller speed as a function of launch speed */
    private final double[] rollerSpeedCoefficients;

    /** The polynomial coefficients to model launch speed as a function of roller speed */
    private final double[] launchSpeedCoefficients;

    private final double maxRollerSpeed;
    private final double maxLaunchSpeed;

    /**
     * Consolidates polynomial regression models to both convert roller speed to launch speed and
     * vice-versa.
     *
     * @param calibrationResults calibration data in the form of a table with distance, roller
     *     speed, and hood angle; the table is assumed to contain data from perfect shots
     * @param maxRollerSpeed the maximum speed of the rollers, in radians per second
     */
    public VelocityLookup(double[][] calibrationResults, double maxRollerSpeed) {
      if (calibrationResults.length < 1) {
        this.rollerSpeedCoefficients = new double[0];
        this.launchSpeedCoefficients = new double[0];

        this.maxRollerSpeed = maxRollerSpeed;
        this.maxLaunchSpeed = evaluate(maxRollerSpeed, launchSpeedCoefficients);
        return;
      }

      DirectLaunchParameters launchParameters =
          new DirectLaunchParameters(MIN_DISTANCE, MAX_SPEED, Math.PI / 4);

      // [LAUNCH SPEED, ROLLER SPEED]
      double[][] calibrationDataTable = new double[calibrationResults.length][];
      for (int index = 0; index < calibrationDataTable.length; index++) {
        launchParameters.setDistance(calibrationResults[index][DISTANCE]);
        launchParameters.setPitch(toPitch(calibrationResults[index][HOOD_ANGLE]));
        ShotOptimizer.optimizeSpeedForAccuracy(
            launchParameters, CALIBRATION_RADIUS, CALIBRATION_DEPTH);
        calibrationDataTable[index] =
            new double[] {launchParameters.speed(), calibrationResults[index][ROLLER_SPEED]};
      }

      // LAUNCH SPEED INPUT --> ROLLER SPEED OUTPUT
      PolynomialRegression rollerSpeedRegression =
          ModelSelector.regression(calibrationDataTable, 0, 1, 3);

      // ROLLER SPEED INPUT --> LAUNCH SPEED OUTPUT
      PolynomialRegression launchSpeedRegression =
          ModelSelector.regression(calibrationDataTable, 1, 0, 3);

      this.rollerSpeedCoefficients = rollerSpeedRegression.getCoefficients();
      this.launchSpeedCoefficients = launchSpeedRegression.getCoefficients();

      this.maxRollerSpeed = maxRollerSpeed;
      this.maxLaunchSpeed = evaluate(maxRollerSpeed, launchSpeedCoefficients);
    }

    /**
     * Whether or not the roller speed exceeds the maximum roller speed.
     *
     * @param rollerSpeed the speed of the rollers, in radians per second
     */
    public boolean inRollerBounds(double rollerSpeed) {
      return rollerSpeed < maxRollerSpeed;
    }

    /**
     * Whether or not the launch speed exceeds the maximum launch speed.
     *
     * @param launchSpeed the speed of the FUEL at launch, in meters per second
     */
    public boolean inLaunchBounds(double launchSpeed) {
      return launchSpeed < maxLaunchSpeed;
    }

    /** The coefficients for the roller speed regression model (launch speed --> roller speed). */
    public double[] rollerSpeedCoefficients() {
      return rollerSpeedCoefficients.clone();
    }

    /** The coefficients for the launch speed regression model (roller speed --> launch speed). */
    public double[] launchSpeedCoefficients() {
      return launchSpeedCoefficients.clone();
    }

    /**
     * The launch speed of the FUEL (meters per second).
     *
     * @param rollerSpeed the speed of the rollers, in radians per second
     */
    public double launchSpeed(double rollerSpeed) {
      return inRollerBounds(rollerSpeed) ? evaluate(rollerSpeed, launchSpeedCoefficients) : 0;
    }

    /**
     * The speed of the rollers (meters per second).
     *
     * @param launchSpeed the launch speed of the FUEL, in meters per second
     */
    public double rollerSpeed(double launchSpeed) {
      return inLaunchBounds(launchSpeed) ? evaluate(launchSpeed, rollerSpeedCoefficients) : 0;
    }

    /**
     * Evaluates the value of a polynomial at a specified x value.
     *
     * @param x the x value to input into the polynomial
     * @param coefficients an array of coefficients describing the polynomial
     */
    public static double evaluate(double x, double[] coefficients) {
      double result = 0;
      for (int degree = 0; degree < coefficients.length; degree++)
        result += coefficients[degree] * Math.pow(x, degree);
      return result;
    }
  }
}
