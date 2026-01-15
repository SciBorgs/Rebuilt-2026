package org.sciborgs1155.robot.hood;

public class NoHood implements HoodIO {

  @Override
  public double angle() {
    return 0;
  }

  @Override
  public void setVoltage(double v) {}

  @Override
  public double velocity() {
    return 0;
  }
}
