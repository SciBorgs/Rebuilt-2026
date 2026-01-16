package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Fake hardware interface for the {@code Turret} subsystem. */
public class NoTurret implements TurretIO {
  @Override
  public void setVoltage(Voltage voltage) {}

  @Override
  public Angle position() {
    return Radians.zero();
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.zero();
  }
}
