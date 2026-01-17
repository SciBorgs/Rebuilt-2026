package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.hood.HoodConstants.GEARING;
import static org.sciborgs1155.robot.hood.HoodConstants.HOOD_RADIUS;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MOI;
import static org.sciborgs1155.robot.hood.HoodConstants.STARTING_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// *hood simulated hardware interface */
public class SimHood implements HoodIO {

  private SingleJointedArmSim sim;

  public SimHood() {

    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GEARING,
            MOI,
            HOOD_RADIUS,
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            STARTING_ANGLE.in(Radians));
  }

  @Override
  public double angle() {
    return sim.getAngleRads() + MIN_ANGLE.in(Radians) + Math.PI / 2;
  }

  @Override
  public void setVoltage(double v) {
    sim.setInputVoltage(v);
    sim.update(v);
  }

  @Override
  public double velocity() {
    return sim.getVelocityRadPerSec();
  }
}
