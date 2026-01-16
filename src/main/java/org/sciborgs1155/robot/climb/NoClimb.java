package org.sciborgs1155.robot.climb;

public class NoClimb implements ClimbIO {

  @Override
  public void close() {}

  @Override
  public void setVoltage(double volt) {}

  @Override
  public double position() {
    return 0.0;
  }

  @Override
  public double velocity() {
    return 0.0;
  }
}
