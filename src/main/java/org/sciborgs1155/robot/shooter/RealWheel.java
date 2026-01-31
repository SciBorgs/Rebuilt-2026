package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealWheel implements WheelIO {
  private final TalonFX motor;

  /** Sets the TalonFX motor configurations */
  public RealWheel() {
    motor = new TalonFX(SHOOTER_MOTOR);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);

    config.Feedback.SensorToMechanismRatio = GEARING * 2 * Math.PI;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motor.getConfigurator().apply(config);
    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
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
