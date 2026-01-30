package org.sciborgs1155.robot.slapdown;

public class NoSlapdown implements SlapdownIO {

  @Override
  public void setArmVoltage(double voltage) {}

  @Override
  public double extensionPosition() {
    return 0;
  }
}
