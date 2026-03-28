package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

/** Fake hardware interface for the {@code Turret} subsystem. */
public class NoTurret implements TurretIO {
  /** Fake hardware interface for the {@code Turret} subsystem. */
  public NoTurret() {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public double voltage() {
    return 0;
  }

  @Override
  public double encoderA() {
    return 0;
  }

  @Override
  public double encoderB() {
    return 0;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void setPosition(Angle angle) {}

  @Override
  public Angle getPosition() {
    return Rotations.zero();
  }
}
