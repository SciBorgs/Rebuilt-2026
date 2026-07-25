package org.sciborgs1155.robot.slapdown;

public interface SlapdownIO extends AutoCloseable {

  /**
   * @param voltage of the intake extending (which is an arm)
   */
  void setVoltage(double voltage);

  /**
   * @return the position of the intake when it is extended
   */
  double position();

  /** Returns the motor current. */
  double current();

  /** Resets the position of the intake to the min angle. */
  void resetPosition();
}
