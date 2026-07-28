package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;

public class MovingShot implements ShootingAlgorithm {

  /** Interpolating tree map key is the distance to target; value is the RPM */
  InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();

  /** Interpolating tree map key is the distance to target; value is the Time of Flight */
  InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

  /** Interpolating tree map key is the distance to target; Value is Hood Angle */
  InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();

  public MovingShot() {

    // Tables have made up values for now

    rpmTable.put(1.0, 1500.0);
    rpmTable.put(2.0, 2500.0);
    rpmTable.put(3.0, 3800.0);

    timeOfFlight.put(3.0, 0.5);
    timeOfFlight.put(9.0, 0.7);
    timeOfFlight.put(18.0, 1.0);

    hoodAngleTable.put(1.0, 0.3);
    hoodAngleTable.put(3.0, 0.5);
    hoodAngleTable.put(9.0, 0.7);
  }

  /**
   * Method to return the desired rpm for flywheel
   *
   * @param distanceToTarget The distance to target in some unit (Have to figure it out)
   * @return the interpolated RPM using rpmTable, a tree map
   */
  public double getRPM(double distanceToTarget) {
    return rpmTable.get(distanceToTarget);
  }

  /**
   * Method to return the desired hood angle for the hood
   *
   * @param distanceToTarget The distance to target in some unit (Have to figure it out)
   * @return the interpolated hood angle using hoodAngleTable, a tree map
   */
  public double getHoodAngle(double distanceToTarget) {
    return hoodAngleTable.get(distanceToTarget);
  }

  @Override
  public ShootingShot shootWhileMoving(
      Translation3d shooterPosition, Translation3d targetPosition, Vector<N2> velocity) {
    double targetX = targetPosition.getX();
    double targetY = targetPosition.getY();

    double robotX = shooterPosition.getX();
    double robotY = shooterPosition.getY();

    double distanceToTarget = targetPosition.getDistance(shooterPosition);

    double velocityX = velocity.get(0, 0);
    double velocityY = velocity.get(1, 0);

    double virtualTargetX = 0;
    double virtualTargetY = 0;

    for (int i = 0; i < 3; i++) {
      double flightTime = timeOfFlight.get(distanceToTarget);

      double driftX = velocityX * flightTime;
      double driftY = velocityY * flightTime;

      virtualTargetX = targetX - driftX;
      virtualTargetY = targetY - driftY;

      Translation3d virtualTarget =
          new Translation3d(virtualTargetX, virtualTargetY, targetPosition.getZ());

      distanceToTarget = virtualTarget.getDistance(shooterPosition);
    }

    double rpm = getRPM(distanceToTarget);
    double hoodAngle = getHoodAngle(distanceToTarget);

    double finalAngle = Math.atan2(virtualTargetY - robotY, virtualTargetX - robotX);

    return new ShootingShot(rpm, finalAngle, hoodAngle);
  }
}