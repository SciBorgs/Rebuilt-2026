package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;

@FunctionalInterface
public interface ShootingAlgorithm {

  // To hold the values
  public record ShootingShot(double RPM, double drivetrainAngle, double hoodAngle) {}

  /**
   * Method to find the rpm, drivetrain angle, hood angle
   *
   * @param shooterPosition Position of shooter as a Translation3d
   * @param targetPosition Position of the target as a Translation3d
   * @param velocity current velocity
   */
  ShootingShot shootWhileMoving(
      Translation3d shooterPosition, Translation3d targetPosition, Vector<N2> velocity);
}
