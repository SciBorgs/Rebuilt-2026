package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.ShootingData.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShot implements ShootingAlgorithm {

  @Override
  public Vector<N3> calculate(Translation3d displacement, Vector<N2> velocity) {

    Translation2d target2D = displacement.toTranslation2d();
    Vector<N2> targetVector = target2D.toVector();
    Vector<N2> projectionDisplacement = targetVector;

    for (int i = 0; i < 3; i++) {

      double currentDistance = projectionDisplacement.norm();
      double flightTime = DISTANCE_TO_TOF.get(currentDistance);

      Vector<N2> drift = velocity.times(flightTime);

      projectionDisplacement =
          VecBuilder.fill(
              targetVector.get(0, 0) - drift.get(0, 0), targetVector.get(1, 0) - drift.get(1, 0));
    }

    Translation2d virtualTarget =
        new Translation2d(projectionDisplacement.get(0, 0), projectionDisplacement.get(1, 0));

    double virtualDistance = virtualTarget.getNorm();

    Translation3d result =
        new Translation3d(
            DISTANCE_TO_RADS.get(virtualDistance) + SIGGYS_CONSTANT.get(),
            new Rotation3d(
                0.0,
                -DISTANCE_TO_HOOD_ANGLE.get(virtualDistance).getRadians(),
                virtualTarget.getAngle().getRadians()));

    return result.toVector();
  }
}
