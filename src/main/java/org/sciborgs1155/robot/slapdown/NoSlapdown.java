package org.sciborgs1155.robot.slapdown;

public class NoSlapdown implements SlapdownIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0;
  }
}
