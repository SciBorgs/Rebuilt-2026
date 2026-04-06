package org.sciborgs1155.lib;

import java.util.Arrays;
import java.util.function.Function;

/** Polynomial Regression with automatic degree selection (thanks Claude). */
@SuppressWarnings("PMD")
public class PolynomialRegression {

  // ===================================================================
  // PolynomialRegression
  // ===================================================================

  private final double[] coefficients;
  private final int degree;
  private final int n;

  public PolynomialRegression(double[] x, double[] y, int degree) {
    if (x == null || y == null) throw new IllegalArgumentException("x and y must not be null.");
    if (x.length != y.length)
      throw new IllegalArgumentException("x and y must have the same length.");
    if (degree < 1) throw new IllegalArgumentException("Degree must be at least 1.");
    if (x.length <= degree)
      throw new IllegalArgumentException(
          "Need at least degree+1 data points; got " + x.length + " for degree " + degree + ".");

    this.degree = degree;
    this.n = x.length;
    int m = degree + 1;

    // Vandermonde design matrix X  (n x m)
    double[][] X = new double[n][m];
    for (int i = 0; i < n; i++) {
      X[i][0] = 1.0;
      for (int j = 1; j < m; j++) X[i][j] = X[i][j - 1] * x[i];
    }

    // Normal equations: A = X^T X,  b = X^T y
    double[][] A = multiply(transpose(X), X);
    double[] b = multiplyMV(transpose(X), y);

    this.coefficients = gaussianElimination(A, b);
  }

  // --- Public API ---

  /** Evaluates the fitted polynomial at xVal (Horner's method). */
  public double predict(double xVal) {
    double result = coefficients[degree];
    for (int i = degree - 1; i >= 0; i--) result = result * xVal + coefficients[i];
    return result;
  }

  public double[] getCoefficients() {
    return coefficients.clone();
  }

  public int getDegree() {
    return degree;
  }

  public int getN() {
    return n;
  }

  // --- Derived statistics ---

  /** Residual Sum of Squares. */
  public double rss(double[] x, double[] y) {
    double ss = 0;
    for (int i = 0; i < x.length; i++) {
      double r = y[i] - predict(x[i]);
      ss += r * r;
    }
    return ss;
  }

  /** Mean Squared Error. */
  public double mse(double[] x, double[] y) {
    return rss(x, y) / x.length;
  }

  /** Plain R². */
  public double rSquared(double[] x, double[] y) {
    double mean = 0;
    for (double v : y) mean += v;
    mean /= y.length;
    double ssTot = 0;
    for (double v : y) {
      double d = v - mean;
      ssTot += d * d;
    }
    return ssTot == 0 ? 1.0 : 1.0 - rss(x, y) / ssTot;
  }

  /**
   * Adjusted R²: 1 - (1-R²)*(n-1)/(n-p-1) Penalizes extra parameters; decreases when a new term
   * does not meaningfully improve fit.
   */
  public double adjustedRSquared(double[] x, double[] y) {
    int p = degree, nPoints = x.length;
    if (nPoints - p - 1 <= 0) return Double.NEGATIVE_INFINITY;
    return 1.0 - (1.0 - rSquared(x, y)) * (nPoints - 1.0) / (nPoints - p - 1.0);
  }

  /**
   * Akaike Information Criterion (Gaussian errors assumed): AIC = n*ln(RSS/n) + 2*(p+1) Mild
   * penalty per parameter — lower is better.
   */
  public double aic(double[] x, double[] y) {
    int p = degree, nPoints = x.length;
    return nPoints * Math.log(rss(x, y) / nPoints) + 2.0 * (p + 1);
  }

  /**
   * Bayesian Information Criterion (Gaussian errors assumed): BIC = n*ln(RSS/n) + (p+1)*ln(n)
   * Stronger penalty than AIC — tends to prefer simpler models. Lower is better.
   */
  public double bic(double[] x, double[] y) {
    int p = degree, nPoints = x.length;
    return nPoints * Math.log(rss(x, y) / nPoints) + (p + 1.0) * Math.log(nPoints);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder("y = ");
    for (int i = 0; i <= degree; i++) {
      double c = coefficients[i];
      if (i == 0) sb.append(String.format("%.6f", c));
      else {
        sb.append(c >= 0 ? " + " : " - ");
        sb.append(String.format("%.6f", Math.abs(c)));
        sb.append(i == 1 ? "x" : "x^" + i);
      }
    }
    return sb.toString();
  }

  // --- Linear-algebra helpers ---

  private static double[][] transpose(double[][] M) {
    int r = M.length, c = M[0].length;
    double[][] T = new double[c][r];
    for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) T[j][i] = M[i][j];
    return T;
  }

  private static double[][] multiply(double[][] A, double[][] B) {
    int r = A.length, inner = B.length, c = B[0].length;
    double[][] C = new double[r][c];
    for (int i = 0; i < r; i++)
      for (int k = 0; k < inner; k++) for (int j = 0; j < c; j++) C[i][j] += A[i][k] * B[k][j];
    return C;
  }

  private static double[] multiplyMV(double[][] A, double[] v) {
    double[] res = new double[A.length];
    for (int i = 0; i < A.length; i++) for (int j = 0; j < v.length; j++) res[i] += A[i][j] * v[j];
    return res;
  }

  private static double[][] deepCopy(double[][] src) {
    double[][] dst = new double[src.length][];
    for (int i = 0; i < src.length; i++) dst[i] = src[i].clone();
    return dst;
  }

  private static double[] gaussianElimination(double[][] A, double[] b) {
    int m = b.length;
    double[][] a = deepCopy(A);
    double[] r = b.clone();
    for (int col = 0; col < m; col++) {
      int pivot = col;
      for (int row = col + 1; row < m; row++)
        if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
      if (Math.abs(a[pivot][col]) < 1e-12)
        throw new ArithmeticException("Matrix is singular — try a lower degree.");
      double[] tmp = a[col];
      a[col] = a[pivot];
      a[pivot] = tmp;
      double tv = r[col];
      r[col] = r[pivot];
      r[pivot] = tv;
      for (int row = col + 1; row < m; row++) {
        double f = a[row][col] / a[col][col];
        r[row] -= f * r[col];
        for (int k = col; k < m; k++) a[row][k] -= f * a[col][k];
      }
    }
    double[] sol = new double[m];
    for (int row = m - 1; row >= 0; row--) {
      double s = r[row];
      for (int k = row + 1; k < m; k++) s -= a[row][k] * sol[k];
      sol[row] = s / a[row][row];
    }
    return sol;
  }

  // ===================================================================
  // ModelSelector — automatic degree selection
  // ===================================================================

  /**
   * Evaluates polynomial degrees from minDegree to maxDegree using four complementary criteria and
   * reports the best degree from each.
   *
   * <pre>
   *   ModelSelector sel = new ModelSelector(x, y, 1, 8);
   *   sel.runAll(5);                      // 5-fold CV + Adj-R², AIC, BIC
   *   int best = sel.crossValidation(5).bestDegree;
   * </pre>
   */
  public static class ModelSelector {

    private final double[] x, y;
    private final int minDegree, maxDegree;

    public ModelSelector(double[] x, double[] y, int minDegree, int maxDegree) {
      if (x.length != y.length)
        throw new IllegalArgumentException("x and y must have the same length.");
      if (minDegree < 1) throw new IllegalArgumentException("minDegree must be >= 1.");
      if (maxDegree < minDegree)
        throw new IllegalArgumentException("maxDegree must be >= minDegree.");
      if (x.length <= maxDegree)
        throw new IllegalArgumentException(
            "Need more data points than maxDegree. Have " + x.length + ", maxDegree=" + maxDegree);
      this.x = x.clone();
      this.y = y.clone();
      this.minDegree = minDegree;
      this.maxDegree = maxDegree;
    }

    // ── 1. k-Fold Cross-Validation ───────────────────────────────────

    /**
     * Selects the degree that minimizes the average hold-out MSE across k folds. Data is shuffled
     * once with a fixed seed for reproducibility.
     *
     * <p>This is the most robust method: it directly measures how well each model generalizes to
     * unseen data.
     *
     * @param k number of folds (5 or 10 are typical choices)
     * @return Result containing the best degree and per-degree CV-MSE scores
     */
    public Result crossValidation(int k) {
      if (k < 2) throw new IllegalArgumentException("k must be at least 2.");
      int n = x.length;
      int[] idx = shuffle(java.util.stream.IntStream.range(0, n).toArray(), 42L);
      double[] cvMSE = new double[maxDegree - minDegree + 1];

      for (int d = minDegree; d <= maxDegree; d++) {
        double totalMSE = 0;
        int validFolds = 0;
        for (int fold = 0; fold < k; fold++) {
          int valStart = fold * n / k, valEnd = (fold + 1) * n / k;
          int valSize = valEnd - valStart, trainSize = n - valSize;
          if (trainSize <= d) continue;

          double[] xTr = new double[trainSize], yTr = new double[trainSize];
          double[] xVl = new double[valSize], yVl = new double[valSize];
          int ti = 0, vi = 0;
          for (int pos = 0; pos < n; pos++) {
            if (pos >= valStart && pos < valEnd) {
              xVl[vi] = x[idx[pos]];
              yVl[vi] = y[idx[pos]];
              vi++;
            } else {
              xTr[ti] = x[idx[pos]];
              yTr[ti] = y[idx[pos]];
              ti++;
            }
          }
          try {
            totalMSE += new PolynomialRegression(xTr, yTr, d).mse(xVl, yVl);
            validFolds++;
          } catch (ArithmeticException ignored) {
          }
        }
        cvMSE[d - minDegree] = validFolds > 0 ? totalMSE / validFolds : Double.MAX_VALUE;
      }
      return buildResult("k-Fold CV (k=" + k + ")", cvMSE, false);
    }

    // ── 2. Adjusted R² ──────────────────────────────────────────────

    /**
     * Selects the degree that maximizes Adjusted R².
     *
     * <p>Adjusted R² = 1 - (1-R²)*(n-1)/(n-p-1). The adjustment penalizes each additional parameter
     * so the metric can decrease when a higher degree does not meaningfully improve fit.
     *
     * @return Result containing the best degree and per-degree Adj-R² scores
     */
    public Result adjustedRSquared() {
      double[] scores = new double[maxDegree - minDegree + 1];
      for (int d = minDegree; d <= maxDegree; d++)
        scores[d - minDegree] = new PolynomialRegression(x, y, d).adjustedRSquared(x, y);
      return buildResult("Adjusted R²", scores, true);
    }

    // ── 3. AIC ──────────────────────────────────────────────────────

    /**
     * Selects the degree that minimizes the Akaike Information Criterion.
     *
     * <p>AIC = n*ln(RSS/n) + 2*(p+1). Applies a mild per-parameter penalty; may allow slightly more
     * complex models than BIC.
     *
     * @return Result containing the best degree and per-degree AIC scores
     */
    public Result aic() {
      double[] scores = new double[maxDegree - minDegree + 1];
      for (int d = minDegree; d <= maxDegree; d++)
        scores[d - minDegree] = new PolynomialRegression(x, y, d).aic(x, y);
      return buildResult("AIC", scores, false);
    }

    // ── 4. BIC ──────────────────────────────────────────────────────

    /**
     * Selects the degree that minimizes the Bayesian Information Criterion.
     *
     * <p>BIC = n*ln(RSS/n) + (p+1)*ln(n). The penalty grows with the dataset size, making BIC
     * prefer simpler models more aggressively than AIC — especially on large datasets.
     *
     * @return Result containing the best degree and per-degree BIC scores
     */
    public Result bic() {
      double[] scores = new double[maxDegree - minDegree + 1];
      for (int d = minDegree; d <= maxDegree; d++)
        scores[d - minDegree] = new PolynomialRegression(x, y, d).bic(x, y);
      return buildResult("BIC", scores, false);
    }

    // ── Convenience: run all four ────────────────────────────────────

    /**
     * Runs all four selection methods and prints a formatted comparison table. Bracket annotations
     * ([CV], [AR2], [AIC], [BIC]) mark the winner of each method.
     *
     * @param k number of folds for cross-validation
     */
    @SuppressWarnings("PMD.SystemPrintln")
    public void runAll(int k) {
      Result cv = crossValidation(k);
      Result adjR = adjustedRSquared();
      Result aicR = aic();
      Result bicR = bic();
      int numDegrees = maxDegree - minDegree + 1;

      System.out.println();
      System.out.println("+=================================================================+");
      System.out.println("|           Degree Selection — Score Comparison Table            |");
      System.out.println("+--------+--------------+--------------+----------+----------+---+");
      System.out.printf(
          "| %-6s | %-12s | %-12s | %-8s | %-8s |   |%n",
          "Degree", "CV-MSE", "Adj-R²", "AIC", "BIC");
      System.out.println("+--------+--------------+--------------+----------+----------+---+");

      for (int i = 0; i < numDegrees; i++) {
        int d = minDegree + i;
        String marker = "";
        if (d == cv.bestDegree) marker += "CV ";
        if (d == adjR.bestDegree) marker += "AR²";
        if (d == aicR.bestDegree) marker += "AIC";
        if (d == bicR.bestDegree) marker += "BIC";
        System.out.printf(
            "| %-6d | %-12s | %-12s | %-8s | %-8s |%-3s|%n",
            d,
            fmt(cv.scores[i]),
            fmt(adjR.scores[i]),
            fmt(aicR.scores[i]),
            fmt(bicR.scores[i]),
            marker);
      }

      System.out.println("+--------+--------------+--------------+----------+----------+---+");
      System.out.printf(
          "|  Best degree →  CV: %d   Adj-R²: %d   AIC: %d   BIC: %d%n",
          cv.bestDegree, adjR.bestDegree, aicR.bestDegree, bicR.bestDegree);
      System.out.println("+=================================================================+");
    }

    // --- Private helpers ---

    private Result buildResult(String method, double[] scores, boolean maximize) {
      int best = 0;
      for (int i = 1; i < scores.length; i++)
        if (maximize ? scores[i] > scores[best] : scores[i] < scores[best]) best = i;
      return new Result(method, minDegree + best, scores);
    }

    private static int[] shuffle(int[] arr, long seed) {
      java.util.Random rng = new java.util.Random(seed);
      for (int i = arr.length - 1; i > 0; i--) {
        int j = rng.nextInt(i + 1);
        int t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
      return arr;
    }

    private static String fmt(double v) {
      if (v == Double.MAX_VALUE) return "N/A";
      if (Math.abs(v) >= 1e4 || (Math.abs(v) < 0.01 && v != 0)) return String.format("%.3e", v);
      return String.format("%.4f", v);
    }

    public static PolynomialRegression regression(
        double[][] dataTable, int xIndex, int yIndex, int maxDegree) {
      double[] x = new double[dataTable.length];
      double[] y = new double[dataTable.length];

      for (int index = 0; index < dataTable.length; index++) {
        x[index] = dataTable[index][xIndex];
        y[index] = dataTable[index][yIndex];
      }

      ModelSelector modelSelector = new ModelSelector(x, y, 1, 5);
      int degree = modelSelector.crossValidation(maxDegree).bestDegree;

      return new PolynomialRegression(x, y, degree);
    }

    public static double[][] dataTable(
        Function<Double, double[]> function, double minimum, double maximum, double increment) {
      int entries = (int) ((maximum - minimum) / increment);
      double[][] table = new double[entries][];

      for (int entry = 0; entry < table.length; entry++)
        table[entry] = function.apply(minimum + entry * increment);

      return table;
    }

    public static record RegressionModel(int degree, double[] coefficients) {
      /** Prediction using horners method, courtesy of Claude AI. */
      public double predict(double x) {
        double result = coefficients[degree];
        for (int index = degree - 1; index >= 0; index--) result = result * x + coefficients[index];
        return result;
      }

      @Override
      public final String toString() {
        return Arrays.toString(coefficients);
      }
    }

    // --- Result record ---

    /** Holds the winning degree and per-degree scores for one selection method. */
    public static class Result {
      /** The method name (e.g. "AIC"). */
      public final String method;

      /** The degree recommended by this method. */
      public final int bestDegree;

      /** Per-degree scores; index 0 corresponds to minDegree. */
      public final double[] scores;

      Result(String method, int bestDegree, double[] scores) {
        this.method = method;
        this.bestDegree = bestDegree;
        this.scores = scores.clone();
      }

      @Override
      public String toString() {
        return String.format("[%s] best degree = %d", method, bestDegree);
      }
    }
  }
}
