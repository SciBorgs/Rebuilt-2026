package org.sciborgs1155.lib.shooting;

import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;
import static org.sciborgs1155.robot.FieldConstants.HUB_HEIGHT;

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
    Vector<N2> target = BLUE_HUB.toTranslation2d().toVector().minus(pose.toVector());
    
    Vector<N2> shot = target.minus(velocity);

    // angle to shoot at goal
    double angle = Math.atan2(shot.get(1), shot.get(0));
    
    // shot speed
    double requiredSpeed = shot.norm();

    // pitch to shoot at goal
    if (launchSpeed < requiredSpeed) return VecBuilder.fill(0,0,0);
    double pitch = Math.acos(shot.norm() * Math.sqrt(9.81 / (BLUE_HUB.getZ() * 2)) / launchSpeed);

    // final X, Y, and Z
    double finalX = Math.cos(angle) * launchSpeed;
    double finalY = Math.sin(angle) * launchSpeed;
    double finalZ = Math.sin(pitch) * launchSpeed;

    return VecBuilder.fill(finalX,finalY,finalZ);
  }
}