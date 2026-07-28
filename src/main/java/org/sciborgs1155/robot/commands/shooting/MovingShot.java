package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.ShootingData.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShot implements ShootingAlgorithm {

  @Override
  public Vector<N3> calculate(Translation3d displacement, Vector<N2> velocity) {

    Vector<N3> projectedDisplacement = displacement.toVector();

    for (int i = 0; i < 3; i++) {
      double flightTime = DISTANCE_TO_TOF.get(displacement.getNorm());

      Vector<N2> drift = velocity.times(flightTime);

      projectedDisplacement =
          projectedDisplacement.plus(VecBuilder.fill(drift.get(0), drift.get(1), 0));
    }

    double distance = projectedDisplacement.norm();

    Translation3d result =
        new Translation3d(
            DISTANCE_TO_RADS.get(distance) + SIGGYS_CONSTANT.get(),
            new Rotation3d(
                0.0,
                -DISTANCE_TO_HOOD_ANGLE.get(distance).getRadians(),
                new Rotation3d(projectedDisplacement).getZ()));
    return result.toVector();
  }
}
