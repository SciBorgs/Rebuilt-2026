package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class NoHood implements HoodIO{

    @Override
    public Angle angle() {
      return Radians.of(0);
    }

    @Override
    public void setVoltage(double v) {}

 
}
