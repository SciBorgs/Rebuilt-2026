package org.sciborgs1155.lib.shooting;

import static org.sciborgs1155.robot.FieldConstants.HUB;

import org.sciborgs1155.robot.Constants.Robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooting implements ShootingAlgorithm {
  // constants
  double launchSpeed = 6.7;

  @Override
  public Vector<N3> calculate(Translation2d pose, Vector<N2> velocity) {
    // target vector (from robot to goal not moving)
    Vector<N2> target = HUB.toTranslation2d().toVector().minus(pose.toVector());

    // angle to shoot at goal
    double angle = Math.atan2(target.get(1), target.get(0));

    // pitch to shoot at goal
    double pitch = Math.acos(target.norm() * Math.sqrt(9.81 / (2 * (HUB.getZ()))) / launchSpeed);

    // final X, Y, and Z
    double finalX = Math.cos(angle) * launchSpeed;
    double finalY = Math.sin(angle) * launchSpeed;
    double finalZ = Math.sin(pitch) * launchSpeed;

    return VecBuilder.fill(finalX,finalY,finalZ);
  }
}
