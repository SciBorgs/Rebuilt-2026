package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.Ports.Shooter.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class RealWheel implements WheelIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;

  public RealWheel() {
    motor = new TalonFX(SHOOTER_MOTOR);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
