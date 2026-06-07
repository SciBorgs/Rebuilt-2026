package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.ShootingData.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class StationaryShooting implements ShootingAlgorithm {

  /* Calculates the firing vector without taking velocity into account.
   *
   * @param displacement Field-relative displacement from the shooter to the target (z unused).
   * @param velocity Full field-relative turret velocity, including translational and rotational
   *     contributions.
   * @return A vector whose direction is the field-relative firing direction and whose magnitude is
   *     shooter wheel speed in rad/s.
   */
  @Override
  public Vector<N3> calculate(Translation3d displacement, Vector<N2> velocity) {
    Translation2d target = displacement.toTranslation2d();
    double distance = target.getNorm();

    Translation3d result =
        new Translation3d(
            DISTANCE_TO_RADS.get(distance) + SIGGYS_CONSTANT.get(),
            new Rotation3d(
                0,
                -DISTANCE_TO_HOOD_ANGLE.get(distance).getRadians(),
                target.getAngle().getRadians()));

    return result.toVector();
  }

  public Vector<N3> calculateHoop(Translation3d displacement, Vector<N2> velocity){
    Translation2d target = displacement.toTranslation2d();
    double distance = target.getNorm();

    Translation3d result =
        new Translation3d(
              DISTANCE_TO_RADS.get(distance) + SIGGYS_CONSTANT.get(),
              new Rotation3d(
                  0,
                  -DISTANCE_TO_HOOD_ANGLE.get(distance).getRadians(),
                  target.getAngle().getRadians()));

      return result.toVector();
  }}
