package org.sciborgs1155.robot.hood;

// *placeholder for when hood is inoperable */
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

  @Override
  public void close() throws Exception {}

  @Override
  public double getVoltage() {
    return 0;
  }
}
