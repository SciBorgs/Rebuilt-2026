package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.DEGREE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CALIBRATION_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CALIBRATION_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.toPitch;

import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.PolynomialRegression;
import org.sciborgs1155.lib.PolynomialRegression.ModelSelector;
import org.sciborgs1155.robot.commands.shooting.ShotOptimizer.DirectLaunchParameters;

/**
 * Consolidates polynomial regression models to both convert roller speed to launch speed and
 * vice-versa.
 */
public final class VelocityLookup {
  /** The polynomial coefficients to model roller speed as a function of launch speed */
  private static double[] lsrsCoefficients = new double[0];

  /** The polynomial coefficients to model launch speed as a function of roller speed */
  private static double[] rslsCoefficients = new double[0];

  private static double maxRollerSpeed, minRollerSpeed;
  private static double maxLaunchSpeed, minLaunchSpeed;

  private static CalibrationEntry[] calibrationEntries;

  /** The progress of the current generation, in entries. */
  private static int progress;

  /** Whether the current generation has finished or not. */
  private static boolean done;

  /**
   * Consolidates polynomial regression models to both convert roller speed to launch speed and
   * vice-versa.
   *
   * @param calibrationResults calibration data in the form of a table with distance, roller speed,
   *     and hood angle; the table is assumed to contain data from perfect shots
   */
  public static void useData(CalibrationEntry[] calibrationResults) {
    if (calibrationResults.length < 1) return;

    done = false;

    DirectLaunchParameters launchParameters =
        new DirectLaunchParameters(MIN_DISTANCE, MAX_SPEED, Math.PI / 4);

    maxRollerSpeed = Double.MIN_VALUE;
    minRollerSpeed = Double.MAX_VALUE;

    maxLaunchSpeed = Double.MIN_VALUE;
    minLaunchSpeed = Double.MAX_VALUE;

    progress = 0;
    calibrationEntries = calibrationResults;

    // [LAUNCH SPEED, ROLLER SPEED]
    double[][] dataTable = new double[calibrationResults.length][];
    for (int index = 0; index < dataTable.length; index++) {
      CalibrationEntry entry = calibrationResults[index];
      launchParameters.setDistance(entry.distance());
      launchParameters.setPitch(toPitch(entry.hoodAngle()));
      ShotOptimizer.optimizeSpeedForAccuracy(
          launchParameters, CALIBRATION_RADIUS, CALIBRATION_DEPTH);
      dataTable[index] = new double[] {launchParameters.speed(), entry.rollerSpeed()};

      if (entry.rollerSpeed() > maxRollerSpeed) maxRollerSpeed = entry.rollerSpeed();
      if (entry.rollerSpeed() < minRollerSpeed) minRollerSpeed = entry.rollerSpeed();

      if (launchParameters.speed() > maxLaunchSpeed) maxLaunchSpeed = launchParameters.speed();
      if (launchParameters.speed() < minLaunchSpeed) minLaunchSpeed = launchParameters.speed();
      progress++;
    }

    // LAUNCH SPEED INPUT --> ROLLER SPEED OUTPUT
    PolynomialRegression lsrsRegression = ModelSelector.regression(dataTable, 0, 1, DEGREE);

    // ROLLER SPEED INPUT --> LAUNCH SPEED OUTPUT
    PolynomialRegression rslsRegression = ModelSelector.regression(dataTable, 1, 0, DEGREE);

    maxLaunchSpeed = evaluate(maxRollerSpeed, rslsCoefficients);
    minLaunchSpeed = evaluate(minRollerSpeed, rslsCoefficients);

    lsrsCoefficients = lsrsRegression.getCoefficients();
    rslsCoefficients = rslsRegression.getCoefficients();

    double lsrsR2 = ModelSelector.rSquared(lsrsRegression, dataTable, 0, 1);
    double rslsR2 = ModelSelector.rSquared(rslsRegression, dataTable, 1, 0);

    done = true;

    LoggingUtils.log(
        "Shooting/Velocity Lookup/Generation/SpeedToRollerSpeed/Coefficients", lsrsCoefficients);
    LoggingUtils.log(
        "Shooting/Velocity Lookup/Generation/RollerSpeedToSpeed/Coefficients", rslsCoefficients);
    LoggingUtils.log("Shooting/Velocity Lookup/Generation/SpeedToRollerSpeed/R^2", lsrsR2);
    LoggingUtils.log("Shooting/Velocity Lookup/Generation/RollerSpeedToSpeed/R^2", rslsR2);

    for (int index = 0; index < calibrationEntries.length; index++)
      if (calibrationResults[index] != null)
        LoggingUtils.log(
            "Shooting/Velocity Lookup/Generation/Calibration Entries/" + index,
            calibrationResults[index].toString());
  }

  /**
   * Whether or not the roller speed exceeds the maximum roller speed.
   *
   * @param rollerSpeed the speed of the rollers, in radians per second
   */
  public static boolean inRollerBounds(double rollerSpeed) {
    return rollerSpeed <= maxRollerSpeed;
  }

  /**
   * Whether or not the launch speed exceeds the maximum launch speed.
   *
   * @param launchSpeed the speed of the FUEL at launch, in meters per second
   */
  public static boolean inLaunchBounds(double launchSpeed) {
    return launchSpeed <= maxLaunchSpeed;
  }

  /** The coefficients for the roller speed regression model (launch speed --> roller speed). */
  public static double[] rollerSpeedCoefficients() {
    return lsrsCoefficients.clone();
  }

  /** The coefficients for the launch speed regression model (roller speed --> launch speed). */
  public static double[] launchSpeedCoefficients() {
    return rslsCoefficients.clone();
  }

  /**
   * The launch speed of the FUEL (meters per second).
   *
   * @param rollerSpeed the speed of the rollers, in radians per second
   */
  public static double launchSpeed(double rollerSpeed) {
    if (rollerSpeed > maxRollerSpeed) return evaluate(maxRollerSpeed, rslsCoefficients);
    if (rollerSpeed < minRollerSpeed) return evaluate(minRollerSpeed, rslsCoefficients);
    return evaluate(rollerSpeed, rslsCoefficients);
  }

  /**
   * The speed of the rollers (radians per second).
   *
   * @param launchSpeed the launch speed of the FUEL, in meters per second
   */
  public static double rollerSpeed(double launchSpeed) {
    if (launchSpeed > maxLaunchSpeed) return evaluate(maxLaunchSpeed, lsrsCoefficients);
    if (launchSpeed < minLaunchSpeed) return evaluate(minLaunchSpeed, lsrsCoefficients);
    return evaluate(launchSpeed, lsrsCoefficients);
  }

  /**
   * Evaluates the value of a polynomial at a specified x value.
   *
   * @param x the x value to input into the polynomial
   * @param coefficients an array of coefficients describing the polynomial
   */
  private static double evaluate(double x, double[] coefficients) {
    double result = 0;
    for (int degree = 0; degree < coefficients.length; degree++)
      result += coefficients[degree] * Math.pow(x, degree);
    return result;
  }

  /**
   * A single calibration entry containing data from a perfect shot. All units are SI and distance
   * is from HUB origin to shooter origin.
   */
  public static record CalibrationEntry(double distance, double rollerSpeed, double hoodAngle) {
    public String toString() {
      return String.format("%.4f %.4f %.4f", distance, rollerSpeed, hoodAngle);
    }
  }

  /** Logs data to NetworkTables. */
  public static void updateLogging() {
    LoggingUtils.log("Shooting/Velocity Lookup/Generation/PROGRESS", progress);
    LoggingUtils.log("Shooting/Velocity Lookup/Generation/COMPLETE", done);
  }
}
