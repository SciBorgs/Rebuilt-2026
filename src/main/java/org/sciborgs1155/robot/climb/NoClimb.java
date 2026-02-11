package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.climb.ClimbConstants.MIN_HEIGHT;

public class NoClimb implements ClimbIO {

  @Override
  public void close() {}

  @Override
  public void setVoltage(double volt) {}

  @Override
  public double position() {
    return MIN_HEIGHT.in(Meters);
  }

  @Override
  public double velocity() {
    return 0.0;
  }
}
