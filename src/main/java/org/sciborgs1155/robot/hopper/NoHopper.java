package org.sciborgs1155.robot.hopper;

public class NoHopper implements HopperIO {

  public NoHopper() {}

  @Override
  public void setVoltage(double volt) {}

  @Override
  public double velocity() {
    return 0.0;
  }

  @Override
  public void close() {}
}
