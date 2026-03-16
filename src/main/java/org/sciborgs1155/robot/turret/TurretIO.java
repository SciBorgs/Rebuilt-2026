package org.sciborgs1155.robot.turret;

/** A hardware interface for the {@code Turret} subsystem. */
public interface TurretIO extends AutoCloseable {
  /**
   * Applies a voltage to the motor.
   *
   * @param double The voltage to apply to the motor in volts.
   */
  void setVoltage(double voltage);

  /**
   * Returns the angular velocity of the motor in radians per second.
   *
   * @return The angular velocity of the motor in radians per second.
   */
  double velocity();

  /**
   * Returns the current angle of the turret in radians.
   *
   * @return The current angle of the turret in radians.
   */  
  double angle();

  double voltage();

  /** Called once per robot loop. Override to update internal state. */
  default void periodic() {}
}
