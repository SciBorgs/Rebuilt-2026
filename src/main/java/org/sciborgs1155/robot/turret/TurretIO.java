package org.sciborgs1155.robot.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
  // TODO: Implement.
  public void setXVoltage(Voltage voltage);

  // TODO: Implement.
  public void setYVoltage(Voltage voltage);

  // TODO: Implement.
  public Angle getXAngle();

  // TODO: Implement.
  public Angle getYAngle();
}
