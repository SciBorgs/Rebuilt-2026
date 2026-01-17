package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.climb.ClimbConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimClimb implements ClimbIO {
  private final ElevatorSim climb =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60(2), WEIGHT.in(Kilograms), SPROCKET_RADIUS.in(Meters), GEARING),
          DCMotor.getKrakenX60(2),
          MIN_HEIGHT.in(Meters),
          MAX_HEIGHT.in(Meters),
          true,
          MIN_HEIGHT.in(Meters));

  public SimClimb() {
    climb.update(PERIOD.in(Seconds));
  }

  @Override
  public void setVoltage(double voltage) {
    climb.setInputVoltage(voltage);
    climb.update(PERIOD.in(Seconds));
  }

  @Override
  public double position() {
    return climb.getPositionMeters();
  }

  @Override
  public double velocity() {
    return climb.getVelocityMetersPerSecond();
  }

  public void resetPosition() {
    climb.setState(0, 0);
  }

  @Override
  public void close() throws Exception {}
  ;
}
