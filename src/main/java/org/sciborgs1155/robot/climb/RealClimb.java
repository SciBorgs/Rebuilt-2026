package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.climb.ClimbConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealClimb implements ClimbIO {
  private final TalonFX motor;

  private TalonFXConfiguration config;

  public RealClimb() {
    motor = new TalonFX(MOTOR_ID);

    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config);
  }

  @Override
  public double position() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volt) {
    motor.setVoltage(volt);
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() {
    motor.close();
  }
}
