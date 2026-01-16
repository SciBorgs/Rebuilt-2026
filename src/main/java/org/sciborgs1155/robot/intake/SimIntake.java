package org.sciborgs1155.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SimIntake implements IntakeIO {
  public static TalonFX rightMotor = new TalonFX(0);
  // public static TalonFX leftMotor = new TalonFX(0);
  public static TalonFX extensionMotor = new TalonFX(0);

  @Override
  public AngularVelocity rollerVelocity() {
    return rightMotor.getVelocity().getValue();
  }

  @Override
  public void setRollerVoltage() {}

  @Override
  public void extend() {}

  @Override
  public Angle extensionPosition() {
    return extensionMotor.getPosition().getValue();
  }

  @Override
  public void retract() {}
}
