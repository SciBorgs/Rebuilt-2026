package org.sciborgs1155.lib.shooting;

import org.sciborgs1155.robot.FieldConstants;

import edu.wpi.first.math.MathUtil;
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

  // goal pos
  double goalX = FieldConstants.HUB.getX();
  double goalY = FieldConstants.HUB.getY();

  double magnusCorrection = 0;

  @Override
  public Vector<N3> calculate(Translation2d pose, Vector<N2> velocity) {

    // robot pos, robot velocity, and project pos
    Vector<N2> robotPos = new Translation2d(pose.getX(), pose.getY()).toVector();
    Vector<N2> projectedPos = robotPos.plus(velocity).times(latency);

    // goal pos
    Vector<N2> goalPos = new Translation2d(goalX, goalY).toVector();

    // target vector (from robot to goal not moving)
    Vector<N2> target = goalPos.minus(projectedPos);

    // shot vector (from robot to goal with moving)
    Vector<N2> shot = target.minus(velocity);

    // angle to shoot at goal
    double angle = Math.atan2(goalY, goalX);

    double adjustedAngle = angle + magnusCorrection * velocity.norm();
    
    // shot speed
    double speed = shot.norm();

    // pitch to shoot at goal
    double pitch = Math.acos(MathUtil.clamp(speed / totalSpeed, 0 ,1));

    // final X, Y, and Z
    double finalX = Math.cos(adjustedAngle) * totalSpeed;
    double finalZ = Math.sin(pitch) * totalSpeed;
    double finalY = Math.sin(adjustedAngle) * totalSpeed;

    Vector<N3> finalVector = new Translation3d(finalX, finalY, finalZ).toVector();

    return finalVector;
  }
}
