package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import static edu.wpi.first.units.Units.Meters;

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
        START_ANGLE.in(Radians)
      );

  @Override
  public void setVoltage(Voltage voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public Angle getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  @Override
  public AngularVelocity getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
  }
}
