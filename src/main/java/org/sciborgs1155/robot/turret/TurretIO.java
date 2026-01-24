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
   * Returns the angular position of the motor.
   *
   * @return The angular position of the motor.
   */
  double position();

  /**
   * Returns the angular velocity of the motor.
   *
   * @return The angular velocity of the motor.
   */
  double velocity();

  /**
   * Returns the voltage of the motor.
   *
   * @return The voltage of the motor.
   */
  double voltage();
}
