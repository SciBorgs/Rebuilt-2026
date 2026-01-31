package org.sciborgs1155.lib.shooting;

import static org.sciborgs1155.robot.FieldConstants.HUB;

import org.sciborgs1155.robot.FieldConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooting implements ShootingAlgorithm {

  InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

  // constants
  double latency = 0.1;
  double totalSpeed = 10;

  double magnusCorrection = 1;

  @Override
  public Vector<N3> calculate(Translation2d pose, Vector<N2> velocity) {
    // robot pos, robot velocity, and project pos, goal po
    Vector<N2> robotPos = pose.toVector();
    Vector<N2> projectedPos = robotPos.plus(velocity).times(latency);
    Vector<N2> goalPos = HUB.toTranslation2d().toVector();

    // target vector (from robot to goal not moving)
    Vector<N2> target = goalPos.minus(projectedPos);

    // shot vector (from robot to goal with moving)
    Vector<N2> shot = target.minus(velocity);

    // angle to shoot at goal
    double angle = Math.atan2(shot.get(1), shot.get(0));
    
    // shot speed
    double speed = shot.norm();

    // pitch to shoot at goal
    double pitch = Math.acos(MathUtil.clamp(speed / totalSpeed, 0 ,1));

    // final X, Y, and Z
    double finalX = Math.cos(angle) * totalSpeed;
    double finalY = Math.sin(angle) * totalSpeed;
    double finalZ = Math.sin(pitch) * totalSpeed;

    return VecBuilder.fill(finalX,finalY,finalZ);
  }
}
