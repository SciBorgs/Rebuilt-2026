package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.ShootingData.DISTANCE_TO_HORIZONTAL_VELOCITY;
import static org.sciborgs1155.robot.Constants.ShootingData.SIGGYS_CONSTANT;
import static org.sciborgs1155.robot.Constants.ShootingData.VELOCITY_TO_HOOD_ANGLE;
import static org.sciborgs1155.robot.Constants.ShootingData.VELOCITY_TO_RADS;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import org.sciborgs1155.lib.Tuning;

public class VelocitySubtraction implements ShootingAlgorithm {
  private final DoubleEntry siggysConstant = Tuning.entry("Robot/shooting/", SIGGYS_CONSTANT);

  @Override
  public Vector<N3> calculate(Translation3d displacement, Vector<N2> velocity) {
    Translation2d target = displacement.toTranslation2d();

    Translation2d stationaryVelocity =
        target.times(DISTANCE_TO_HORIZONTAL_VELOCITY.get(target.getNorm()) / target.getNorm());
    Translation2d desiredVelocity = stationaryVelocity.minus(new Translation2d(velocity));

    double desiredSpeed = desiredVelocity.getNorm();

    Translation3d result =
        new Translation3d(
            VELOCITY_TO_RADS.get(desiredSpeed) + siggysConstant.get(),
            new Rotation3d(
                0,
                -VELOCITY_TO_HOOD_ANGLE.get(desiredSpeed).getRadians(),
                desiredVelocity.getAngle().getRadians()));

    return result.toVector();
  }
}
