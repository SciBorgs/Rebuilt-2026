package org.sciborgs1155.robot.intake;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
  /**
   * @return the velocity of the rollers
   */
  public AngularVelocity rollerVelocity();

  /** sets the voltage of the rollers */
  public void setRollerVoltage();

  /**
   * @param voltage of the intake extending (which is an arm)
   */
  public void setArmVoltage(double voltage);

  /**
   * @return the position of the intake when it is extended
   */
  public double extensionPosition();
}
