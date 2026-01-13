package org.sciborgs1155.robot.hood;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {

  public Angle angle();

  public void setVoltage(double v);

  public double velocity();
}
