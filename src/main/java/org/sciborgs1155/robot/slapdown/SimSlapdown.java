package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimSlapdown implements SlapdownIO, AutoCloseable {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX,
          GEARING,
          MOI,
          LENGTH,
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          START_ANGLE.in(Radians),
          measurementStDevs);

  @Override
  /** set the voltage of the slapdown */
  public void setArmVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  /** get the position of the slapdown */
  public double extensionPosition() {
    return sim.getAngleRads();
  }

  @Override
  public void close() throws Exception {
    sim.setInputVoltage(0);
  }
}
