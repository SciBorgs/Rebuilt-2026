package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulated hardware interface for the {@code Turret} subsystem. */
public class SimTurret implements TurretIO {
  /** Simulated servo motor representing the turret. */
  private final SingleJointedArmSim simulation =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          GEAR_RATIO,
          MOI.in(KilogramSquareMeters),
          TURRET_LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          false,
          START_ANGLE.in(Radians));

  @Override
  public void setVoltage(Voltage voltage) {
    simulation.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public Angle position() {
    return Radians.of(simulation.getAngleRads());
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.of(simulation.getVelocityRadPerSec());
  }
}
