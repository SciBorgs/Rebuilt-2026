package org.sciborgs1155.robot.turret;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  public static Turret create() {
    return Robot.isReal() ? 
      new Turret(new RealTurret()) : 
      new Turret(new SimTurret());
  }

  public static Turret none() {
    return new Turret(new NoTurret());
  }

  private final TurretIO motor;

  public Turret(TurretIO turretIO) {
    motor = turretIO;
  }
}
