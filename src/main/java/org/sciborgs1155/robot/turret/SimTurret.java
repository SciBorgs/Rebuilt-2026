package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
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
          DCMotor.getKrakenX60(1), // GEARBOX
          GEAR_RATIO, // GEARING
          MOI.in(KilogramSquareMeters), // MOMENT OF INERTIA
          TURRET_LENGTH.in(Meters), // ARM LENGTH
          MIN_ANGLE.in(Radians), // MINIMUM ANGLE
          MAX_ANGLE.in(Radians), // MAXIMUM ANGLE
          false, // GRAVITY ENABLED
          START_ANGLE.in(Radians)); // STARTING ANGLE

  /** Simulated hardware interface for the {@code Turret} subsystem. */
  public SimTurret() {}

  @Override
  public void setVoltage(Voltage voltage) {
    simulation.setInputVoltage(voltage.in(Volts));
    simulation.update(PERIOD.in(Seconds));
  }

  @Override
  public Angle position() {
    return Radians.of(simulation.getAngleRads() / SENSOR_TO_MECHANISM_RATIO);
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.of(simulation.getVelocityRadPerSec());
  }

  @Override
  public Voltage voltage() {
    return Volts.of(simulation.getInput(0));
  }

  @Override
  public void close() throws Exception {}
}
