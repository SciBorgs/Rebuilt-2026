package org.sciborgs1155.lib.shooting;

import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import org.sciborgs1155.robot.FieldConstants.Hub;

public class BasedShootingAlgorithm implements ShootingAlgorithm {
  private static final double APEX_HEIGHT_ABOVE_TARGET = 2.0;
  private static final double GRAVITY = 9.80665;

  @Override
  public Vector<N3> calculate(Translation3d pose, Vector<N2> velocity) {
    double dx = Hub.TOP_CENTER_POINT.getX() - pose.getX();
    double dy = Hub.TOP_CENTER_POINT.getY() - pose.getY();
    double horizontalDist = Math.hypot(dx, dy);

    double zStart = pose.getZ() + ROBOT_TO_SHOOTER.getZ();
    double zTarget = Hub.TOP_CENTER_POINT.getZ();

    double zApex = Math.max(zStart, zTarget) + APEX_HEIGHT_ABOVE_TARGET;

    double vZ = Math.sqrt(2 * GRAVITY * (zApex - zStart));

    double timeToPeak = vZ / GRAVITY;
    double timeToFall = Math.sqrt(2 * (zApex - zTarget) / GRAVITY);
    double totalTime = timeToPeak + timeToFall;

    double vXY = horizontalDist / totalTime;

    double unitX = dx / horizontalDist;
    double unitY = dy / horizontalDist;

    return VecBuilder.fill(
        (unitX * vXY) - velocity.get(0, 0), (unitY * vXY) - velocity.get(1, 0), vZ);
  }
}
