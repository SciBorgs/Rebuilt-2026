package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.hood.HoodConstants.GEARING;
import static org.sciborgs1155.robot.hood.HoodConstants.HOOD_LENGTH;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MOI;
import static org.sciborgs1155.robot.hood.HoodConstants.STARTING_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimHood implements HoodIO {

  private SingleJointedArmSim sim;

  public SimHood() {

    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GEARING,
            MOI,
            HOOD_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            true,
            STARTING_ANGLE);
  }

  @Override
  public Angle angle() {
    return Radians.of(sim.getAngleRads());
  }

  @Override
  public void setVoltage(double v) {
    sim.setInputVoltage(v);
  }

  @Override
  public double velocity() {
    return sim.getVelocityRadPerSec();
  }
}
