package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class NoTurret implements TurretIO {
  @Override
  public void setXVoltage() {}

  @Override
  public void setYVoltage() {}

  @Override
  public Angle getXAngle() {
    return Degrees.of(0);
  }

  @Override
  public Angle getYAngle() {
    return Degrees.of(0);
  }
}
