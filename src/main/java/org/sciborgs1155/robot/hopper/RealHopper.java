package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Constants.INTAKE_CANIVORE;
import static org.sciborgs1155.robot.Ports.Hopper.MOTOR;
import static org.sciborgs1155.robot.hopper.HopperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealHopper implements HopperIO {
  private final TalonFX motor;

  /** creates a HopperIO with a real motor implementation */
  public RealHopper() {
    motor = new TalonFX(MOTOR, INTAKE_CANIVORE);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEARING;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void setVoltage(double volt) {
    motor.setVoltage(volt);
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void close() {
    motor.close();
  }
}
