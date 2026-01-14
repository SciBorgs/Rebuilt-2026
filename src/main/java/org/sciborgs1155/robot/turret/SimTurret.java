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

public class SimTurret implements TurretIO {
  private final SingleJointedArmSim sim =
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
    sim.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public Angle getPosition() {
    return Radians.of(sim.getAngleRads());
  }

  @Override
  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(sim.getVelocityRadPerSec());
  }
}
