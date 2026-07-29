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

    Vector<N3> targetVector = displacement.toVector();
    Vector<N3> projectionDisplacement = targetVector;

    for (int i = 0; i < 3; i++) {

    double currentDistance = projectionDisplacement.norm();
    double flightTime =
        DISTANCE_TO_TOF.get(currentDistance);

    Vector<N2> drift = velocity.times(flightTime);

    projectionDisplacement = VecBuilder.fill(
      targetVector.get(0,0) - drift.get(0,0),
      targetVector.get(1, 0) - drift.get(1,0),
      targetVector.get(2,0)
    );
    }

    double finalDistance = projectionDisplacement.norm();

    Translation3d result =
        new Translation3d(
            DISTANCE_TO_RADS.get(finalDistance) + SIGGYS_CONSTANT.get(),
            new Rotation3d(
                0.0,
                -DISTANCE_TO_HOOD_ANGLE.get(finalDistance).getRadians(), 0.0));
    
   return result.toVector();
  }
}
