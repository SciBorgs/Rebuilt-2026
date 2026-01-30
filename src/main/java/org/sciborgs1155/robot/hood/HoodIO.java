package org.sciborgs1155.robot.hood;

public interface HoodIO extends AutoCloseable {

  /**
   * gets the current angle of the hood
   *
   * @return angle in rads
   */
  double angle();

  /**
   * sets the voltage of the hood motor
   *
   * @param v
   */
  void setVoltage(double v);

  /**
   * gets the current velocity of the hood
   *
   * @return vel in rads/sec
   */
  double velocity();

  @Override
  void close() throws Exception;
}