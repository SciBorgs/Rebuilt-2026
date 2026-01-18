package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class NoIntake implements IntakeIO {

    @Override
    public AngularVelocity rollerVelocity() {
        return RadiansPerSecond.of(0);
    }

    @Override
    public void setRollerVoltage() {}

    @Override
    public void setArmVoltage() {}

    @Override
    public double extensionPosition() {
        return 0;
    }

}
