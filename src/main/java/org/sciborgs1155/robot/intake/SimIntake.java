package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimIntake implements IntakeIO {
  /*public static TalonFX rightMotor = new TalonFX(0);
  // public static TalonFX leftMotor = new TalonFX(0);
  public static TalonFX extensionMotor = new TalonFX(0);*/
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          IntakeConstants.GEARBOX,
          IntakeConstants.GEARING,
          IntakeConstants.MOI,
          IntakeConstants.LENGTH,
          IntakeConstants.MIN_ANGLE.in(Radians),
          IntakeConstants.MAX_ANGLE.in(Radians),
          true,
          IntakeConstants.START_ANGLE,
          null);

  @Override
  public AngularVelocity rollerVelocity() {
    return RadiansPerSecond.of(0);
  }

  @Override
  public void setRollerVoltage() {}

  @Override
  public void setArmVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public double extensionPosition() {
    return sim.getAngleRads();
  }
}
