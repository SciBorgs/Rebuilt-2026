package org.sciborgs1155.robot.hopper;

public class NoHopper implements HopperIO {
  /** sets the voltage of the hopper motor */
  @Override
  public void setVoltage(double volt) {}

  /** returns the velocity in radians per second */
  @Override
  public double velocity() {
    return 0.0;
  }

  @Override
  public void close() {}
}
