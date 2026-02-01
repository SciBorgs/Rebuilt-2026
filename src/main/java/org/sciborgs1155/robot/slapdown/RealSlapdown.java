package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Slapdown.*;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealSlapdown implements SlapdownIO, AutoCloseable {
  private final TalonFX motor;

  /** Configures the motors */
  public RealSlapdown() {
    motor = new TalonFX(EXTENSION);

    // Rollers - Kraken X44
    // Check how integrated encoder works

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // TODO Define confi
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
  }

  @Override
  // extending/retracting the intake
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  // getting position of the intake (extended or retracted)
  public double position() {
    return motor.getPosition().getValue().in(Radians);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
