package org.sciborgs1155.robot.hood;

import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// *hood simulated hardware interface */
public class SimHood implements HoodIO {

  private final SingleJointedArmSim sim;

  /** constructor for sim hood */
  public SimHood() {

    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            GEARING,
            MOI,
            HOOD_RADIUS.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            STARTING_ANGLE.in(Radians));
  }

  @Override
  public double angle() {
    return sim.getAngleRads();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
    System.out.println("Hood Voltage: " + volts);
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return sim.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
