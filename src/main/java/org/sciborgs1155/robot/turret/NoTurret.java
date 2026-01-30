package org.sciborgs1155.robot.turret;

/** Fake hardware interface for the {@code Turret} subsystem. */
public class NoTurret implements TurretIO {
  /** Fake hardware interface for the {@code Turret} subsystem. */
  public NoTurret() {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0;
  }

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
