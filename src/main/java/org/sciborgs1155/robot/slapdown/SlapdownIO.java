package org.sciborgs1155.robot.slapdown;

public interface SlapdownIO {

  /**
   * @param voltage of the intake extending (which is an arm)
   */
  public void setArmVoltage(double voltage);

  /**
   * @return the position of the intake when it is extended
   */
  public double extensionPosition();
}
