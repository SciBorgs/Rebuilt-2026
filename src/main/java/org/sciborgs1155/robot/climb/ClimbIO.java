package org.sciborgs1155.robot.climb;

public interface ClimbIO extends AutoCloseable {
  /**
   * Sets the voltage of the motor.
   *
   * @param volt The voltage to set the motor to.
   */
  void setVoltage(double volt);

  /**
   * Returns the position of the elevator.
   *
   * @return The position in meters.
   */
  double position();

  /**
   * Returns velocity of elevator in m/s.
   *
   * @return Velocity in m/s.
   */
  double velocity();
}
