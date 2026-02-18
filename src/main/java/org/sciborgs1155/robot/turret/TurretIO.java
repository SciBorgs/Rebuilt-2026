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
   * Returns the angular position of the motor in radians.
   *
   * @return The angular position of the motor in radians.
   */
  double position();

  /**
   * Returns the angular velocity of the motor in radians per second.
   *
   * @return The angular velocity of the motor in radians per second.
   */
  double velocity();

  /**
   * Returns the angular position of encoder A in rotations.
   *
   * @return The angular position of encoder A in rotations.
   */
  double encoderA();

  double rawA();

  /**
   * Returns the angular position of encoder B in rotations.
   *
   * @return The angular position of encoder B in rotations.
   */
  double encoderB();

  /** Called once per robot loop. Override to update internal state. */
  default void periodic() {}
}
