package org.sciborgs1155.robot.slapdown;

public interface SlapdownIO {

  /**
   * @param voltage of the intake extending (which is an arm)
   */
  void setVoltage(double voltage);

  /**
   * @return the position of the intake when it is extended
   */
  double position();
}
