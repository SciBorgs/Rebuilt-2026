package org.sciborgs1155.robot.climb;

public interface ClimbIO extends AutoCloseable {
  /**
   * sets the voltage of the motor
   *
   * @param volt the voltage to set the motor to
   */
  public void setVoltage(double volt);

  /**
   * returns the position of the elevator
   *
   * @return position in meters
   */
  public double position();

  /**
   * returns velocity of elevator in m/s
   *
   * @return velocity in m/s
   */
  public double velocity();
}
