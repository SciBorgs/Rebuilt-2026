package org.sciborgs1155.lib.shooting;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;
import static org.sciborgs1155.robot.FieldConstants.HUB_HEIGHT;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooter implements ShootingAlgorithm {

  // constants
  double launchSpeed = 6.7;

  @Override
  public Vector<N3> calculate(Translation3d pose, Vector<N2> velocity) {
    // target vector (from robot to goal not moving)
    Vector<N2> target = BLUE_HUB.toVector().minus(pose.toTranslation2d().toVector());

    Vector<N2> shot = target.minus(velocity);

    // angle to shoot at goal
    double angle = Math.atan2(shot.get(1), shot.get(0));

    // shot speed
    double requiredSpeed = shot.norm();

    // pitch to shoot at goal
    if (launchSpeed < requiredSpeed) return VecBuilder.fill(0, 0, 0);
    double pitch =
        Math.acos(shot.norm() * Math.sqrt(9.81 / (HUB_HEIGHT.in(Meters) * 2)) / launchSpeed);

    // final X, Y, and Z
    double finalX = Math.cos(angle) * launchSpeed;
    double finalY = Math.sin(angle) * launchSpeed;
    double finalZ = Math.sin(pitch) * launchSpeed;

    return VecBuilder.fill(finalX, finalY, finalZ);
  }
}
