package org.sciborgs1155.robot.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
  public void setVoltage(Voltage voltage);

  public Angle getPosition();

  public AngularVelocity getVelocity();
}
