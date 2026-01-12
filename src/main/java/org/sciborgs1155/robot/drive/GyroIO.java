package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N2;

/** Generalized gyroscope. Pigeon2, Navx, and SimGyro are to be implemented. */
public interface GyroIO extends AutoCloseable {
  /** Calibrates the gyroscope. Pigeon2 does not need to do anything here. */
  default void calibrate() {}

  /** Returns the rate of rotation. */
  double rate();

  /** Returns the heading of the robot as a Rotation2d. */
  default Rotation2d rotation2d() {
    return rotation3d().toRotation2d();
  }

  /** Returns the heading of the robot as a Rotation3d. */
  Rotation3d rotation3d();

  /**
   * Returns the list of headings for the last tick, from a faster thread. [[headings],
   * [timestamps]]
   */
  double[][] odometryData();

  /** Returns the acceleration of the robot as a Vector. */
  Vector<N2> acceleration();

  /** Resets heading to 0 */
  void reset(Rotation2d heading);
}
