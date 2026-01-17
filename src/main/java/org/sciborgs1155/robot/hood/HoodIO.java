package org.sciborgs1155.robot.hood;

public interface HoodIO {

  /**
   * gets the current angle of the hood
   * @return angle in rads
   */
  public double angle();


  /**
   * sets the voltage of the hood motor
   * @param v
   */
  public void setVoltage(double v);


  /**
   * gets the current velocity of the hood
   * @return vel in rads/sec
   */
  public double velocity();
}
