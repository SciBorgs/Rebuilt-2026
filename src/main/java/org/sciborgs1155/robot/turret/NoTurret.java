package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Fake hardware interface for the {@code Turret} subsystem. */
public class NoTurret implements TurretIO {
  /** Fake hardware interface for the {@code Turret} subsystem. */
  public NoTurret() {}

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

  @Override
  public Voltage voltage() {
    return Volts.of(0);
  }

  @Override
  public void close() throws Exception {}
}
