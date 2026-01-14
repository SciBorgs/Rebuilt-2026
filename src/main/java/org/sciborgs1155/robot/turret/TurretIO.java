package org.sciborgs1155.robot.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
  // TODO: Implement.
  public void setVoltage(Voltage voltage);

  // TODO: Implement.
  public Angle getPosition();

  // TODO: Implement.
  public AngularVelocity getVelocity();
}
