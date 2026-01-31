package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.climb.ClimbConstants.*;

import org.sciborgs1155.lib.SpringLoadedClimbSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class SimClimb implements ClimbIO {
  private final SpringLoadedClimbSim climb =
      new SpringLoadedClimbSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60(2), WEIGHT.in(Kilograms), SPROCKET_RADIUS.in(Meters), GEARING),
          DCMotor.getKrakenX60(2),
          MIN_HEIGHT.in(Meters),
          MAX_HEIGHT.in(Meters),
          SPRING_ACCELERATION.in(MetersPerSecondPerSecond), 
          STARTING_HEIGHT.in(Meters));

  /** Constructor of the climb simulator */
  public SimClimb() {
    climb.update(PERIOD.in(Seconds));
  }

  @Override
  public void setVoltage(double voltage) {
    climb.setInputVoltage(voltage);
    climb.update(PERIOD.in(Seconds));
    System.out.println(voltage);
  }

  @Override
  public double position() {
    return climb.getPositionMeters();
  }

  @Override
  public double velocity() {
    return climb.getVelocityMetersPerSecond();
  }

  /** Sets sim climb position to 0 and velocity to 0 */
  public void resetPosition() {
    climb.setState(0, 0);
  }

  @Override
  public void close() throws Exception {}
}
