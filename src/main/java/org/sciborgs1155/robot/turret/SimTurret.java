package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulated hardware interface for the {@code Turret} subsystem. */
public class SimTurret implements TurretIO {
  /** Simulated servo motor representing the turret. */
  private final SingleJointedArmSim simulation =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), // GEARBOX
          GEAR_RATIO, // GEARING
          MOI.in(KilogramSquareMeters), // MOMENT OF INERTIA
          TURRET_RADIUS.in(Meters), // TURRET RADIUS
          MIN_ANGLE.in(Radians), // MINIMUM ANGLE
          MAX_ANGLE.in(Radians), // MAXIMUM ANGLE
          false, // GRAVITY DISBLAED
          START_ANGLE.in(Radians)); // STARTING ANGLE

  /** Turret angle in radians (mechanism space). */
  @Override
  public double angle() {
    return simulation.getAngleRads();
  }

  @Override
  public void setVoltage(double voltage) {
    simulation.setInputVoltage(voltage);
    simulation.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return simulation.getVelocityRadPerSec();
  }

  @Override
  public double voltage() {
    return simulation.getInput(0);
  }

  @Override
  public void close() throws Exception {}
}
