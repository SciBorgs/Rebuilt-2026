package org.sciborgs1155.lib.shooting;

import static org.sciborgs1155.robot.FieldConstants.HUB;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooting implements ShootingAlgorithm {

  InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

  // constants
  double latency = 0.1;
  double exitSpeed = 10;

  double magnusCorrection = 1;

  @Override
  public Vector<N3> calculate(Translation2d pose, Vector<N2> velocity) {
    // target vector (from robot to goal not moving)
    Vector<N2> target = HUB.toTranslation2d().toVector().minus(pose.toVector());

    // angle to shoot at goal
    double angle = Math.atan2(target.get(1), target.get(0));
    
    // shot speed
    double requiredSpeed = target.norm();

    // pitch to shoot at goal
    double pitch = Math.acos(MathUtil.clamp(requiredSpeed / exitSpeed, 0 ,1));

    // final X, Y, and Z
    double finalX = Math.cos(angle) * exitSpeed;
    double finalY = Math.sin(angle) * exitSpeed;
    double finalZ = Math.sin(pitch) * exitSpeed;

    return VecBuilder.fill(finalX,finalY,finalZ);
  }
}
