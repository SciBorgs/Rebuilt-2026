package org.sciborgs1155.robot.indexer;

public class NoIndexer implements IndexerIO {
  /** sets the voltage of the indexer motor */
  @Override
  public void setVoltage(double volt) {}

  /** returns the velocity of the motor in radians per second */
  @Override
  public double velocity() {
    return 0.0;
  }

  @Override
  public void close() {}
}
