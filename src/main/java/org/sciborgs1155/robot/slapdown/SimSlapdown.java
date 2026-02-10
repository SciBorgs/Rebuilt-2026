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
          MEASUREMENT_STDEVS);

  // set the voltage of the slapdown
  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Seconds));
  }

  // get the position of the slapdown
  @Override
  public double position() {
    return sim.getAngleRads();
  }

  @Override
  public void close() throws Exception {
    sim.setInputVoltage(0);
  }
}
