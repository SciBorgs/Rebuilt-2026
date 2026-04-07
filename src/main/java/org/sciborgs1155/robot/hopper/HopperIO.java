package org.sciborgs1155.robot.hopper;

public interface HopperIO extends AutoCloseable {

  /**
   * Sets the voltage of the motor.
   *
   * @param volt The voltage to set the motor to.
   */
  void setVoltage(double volt);
  
  /**
   * Returns velocity of hopper in radians/s.
   *
   * @return Velocity in radians/s.
   */
  double velocity();
}