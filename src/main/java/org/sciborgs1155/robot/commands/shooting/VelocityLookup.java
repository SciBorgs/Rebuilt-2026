package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.HOOD_ANGLE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.ROLLER_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MAX_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CALIBRATION_DEPTH;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.CALIBRATION_RADIUS;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.toPitch;

import org.sciborgs1155.lib.PolynomialRegression;
import org.sciborgs1155.lib.PolynomialRegression.ModelSelector;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.DirectLaunchParameters;

/**
 * Consolidates polynomial regression models to both convert roller speed to launch speed and
 * vice-versa.
 */
public class VelocityLookup {
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
   * @param calibrationResults calibration data in the form of a table with distance, roller speed,
   *     and hood angle; the table is assumed to contain data from perfect shots
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
   * The speed of the rollers (radians per second).
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
