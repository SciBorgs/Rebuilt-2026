package org.sciborgs1155.robot.turret;

import edu.wpi.first.math.MathUtil;
import java.util.OptionalDouble;

/**
 * Custom Chinese Remainder Theorem solver for dual absolute encoder turret position resolution.
 *
 * <p>Given two absolute encoders geared at coprime ratios to the mechanism, uniquely determines the
 * mechanism's absolute position in rotations within the valid range.
 */
public class CustomCRT {
  /** Encoder A turns per mechanism rotation. */
  private final double rA;

  /** Encoder B turns per mechanism rotation. */
  private final double rB;

  private final double minRot;
  private final double maxRot;
  private final double toleranceRot;
  private final boolean invertA;
  private final boolean invertB;
  private final double offsetA;
  private final double offsetB;

  /**
   * @param rA encoder A turns per mechanism rotation (e.g. turretGearing / encoderAGearing)
   * @param rB encoder B turns per mechanism rotation
   * @param minRot minimum mechanism position in rotations
   * @param maxRot maximum mechanism position in rotations
   * @param toleranceRot acceptable match tolerance in rotations
   * @param invertA whether to invert encoder A reading
   * @param invertB whether to invert encoder B reading
   * @param offsetA encoder A zero offset in rotations (subtracted before inversion)
   * @param offsetB encoder B zero offset in rotations
   */
  public CustomCRT(
      double rA,
      double rB,
      double minRot,
      double maxRot,
      double toleranceRot,
      boolean invertA,
      boolean invertB,
      double offsetA,
      double offsetB) {
    this.rA = rA;
    this.rB = rB;
    this.minRot = minRot;
    this.maxRot = maxRot;
    this.toleranceRot = toleranceRot;
    this.invertA = invertA;
    this.invertB = invertB;
    this.offsetA = offsetA;
    this.offsetB = offsetB;
  }

  /** Applies offset then inversion to a raw 0-1 encoder reading. */
  private double process(double raw, double offset, boolean invert) {
    double v = MathUtil.inputModulus(raw - offset, 0.0, 1.0);
    return invert ? MathUtil.inputModulus(1.0 - v, 0.0, 1.0) : v;
  }

  /**
   * Attempts to solve the mechanism position from two raw absolute encoder readings.
   *
   * <p>Iterates over all mechanism positions consistent with encoder A, then checks which one also
   * satisfies encoder B within tolerance.
   *
   * @param rawA raw encoder A reading (0-1 rotations)
   * @param rawB raw encoder B reading (0-1 rotations)
   * @return mechanism position in rotations, or empty if no valid solution found
   */
  public OptionalDouble solve(double rawA, double rawB) {
    double a = process(rawA, offsetA, invertA);
    double b = process(rawB, offsetB, invertB);

    // θ = (a + n) / rA satisfies encoder A for any integer n.
    // ±1 buffer guards against floating point precision in the boundary computation.
    int nMin = (int) Math.floor(minRot * rA - a) - 1;
    int nMax = (int) Math.ceil(maxRot * rA - a) + 1;

    OptionalDouble result = OptionalDouble.empty();

    for (int n = nMin; n <= nMax; n++) {
      double candidate = (a + n) / rA;
      if (candidate < minRot || candidate > maxRot) continue;

      double predictedB = MathUtil.inputModulus(candidate * rB, 0.0, 1.0);
      // Circular distance on encoder B face, converted to mechanism rotations for comparison.
      double errorRot = Math.abs(MathUtil.inputModulus(predictedB - b + 0.5, 0.0, 1.0) - 0.5) / rB;

      if (errorRot <= toleranceRot) {
        if (result.isPresent()) {
          // Two candidates passed — ambiguous, cannot safely seed.
          return OptionalDouble.empty();
        }
        result = OptionalDouble.of(candidate);
      }
    }

    return result;
  }
}
