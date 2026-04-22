package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.ShootingData.*;

import org.sciborgs1155.lib.Tuning;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import static org.sciborgs1155.robot.Constants.ShootingData.SIGGYS_CONSTANT;

import org.sciborgs1155.robot.commands.Shooting;

/**
 * Implements {@link ShootingAlgorithm} using iterative time-of-flight refinement.
 *
 * <p>The {@code pose} parameter to {@link #calculate} is the field-relative displacement from the
 * shooter to the target. The {@code velocity} parameter is the full field-relative turret velocity,
 * including both translational and rotational contributions (translational + omega * r,
 * pre-computed by the caller).
 *
 * <p>Returns a {@code Vector<N3>} whose direction is the field-relative firing direction and whose
 * magnitude is the shooter wheel speed in rad/s.
 */
public class TOFIteration implements ShootingAlgorithm {
  private static final int ITERATIONS = 10;

  /**
   * Calculates the firing vector by iteratively refining the time of flight.
   *
   * @param displacement Field-relative displacement from the shooter to the target (z unused).
   * @param velocity Full field-relative turret velocity, including translational and rotational
   *     contributions.
   * @return A vector whose direction is the field-relative firing direction and whose magnitude is
   *     shooter wheel speed in rad/s.
   */
  @Override
  public Vector<N3> calculate(Translation3d displacement, Vector<N2> velocity, Vector<N2> accel) {
    Translation2d target = displacement.toTranslation2d();
    Translation2d lookAhead = new Translation2d();

    for (int i = 0; i < ITERATIONS; i++) {
      double distance = target.getDistance(lookAhead);
      double tof = DISTANCE_TO_TOF.get(distance);
      lookAhead =
          new Translation2d(
              velocity.times(tof).plus(accel.times(Math.pow(Shooting.LATENCY_TIME.get(), 2) / 2)));
    }

    double distance = target.getDistance(lookAhead);

    Translation3d result =
        new Translation3d(
            DISTANCE_TO_RADS.get(distance) + SIGGYS_CONSTANT.get(),
            new Rotation3d(
                0,
                -DISTANCE_TO_HOOD_ANGLE.get(distance).getRadians(),
                target.minus(lookAhead).getAngle().getRadians()));

    return result.toVector();
  }

  @Override
  public Vector<N3> calculate(Translation3d displacement, Vector<N2> velocity) {
    return calculate(displacement, velocity, VecBuilder.fill(0, 0));
  }
}
