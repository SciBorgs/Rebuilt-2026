package org.sciborgs1155.robot.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** A hardware interface for the {@code Turret} subsystem. */
public interface TurretIO extends AutoCloseable {
  /**
   * Applies a voltage to the motor.
   *
   * @param voltage The voltage to apply to the motor.
   */
  public void setVoltage(Voltage voltage);

  /**
   * Returns the angular position of the motor.
   *
   * @return The angular position of the motor.
   */
  public Angle position();

  /**
   * Returns the angular velocity of the motor.
   *
   * @return The angular velocity of the motor.
   */
  public AngularVelocity velocity();
}
