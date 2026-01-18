package org.sciborgs1155.robot.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
  public AngularVelocity rollerVelocity();

  public void setRollerVoltage();

  public void setArmVoltage();

  public double extensionPosition();
}
