package org.sciborgs1155.robot.shooter;

public interface WheelIO extends AutoCloseable {
  /**
   * Sets the voltage for the flywheel.
   *
   * @param voltage The voltage
   */
  public void setVoltage(double voltage);

  /**
   * The velocity of the flywheel, in radians per seconds.
   *
   * @return The velocity of the flywheel, in radians per seconds.
   */
  public double velocity();
}
