package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Intake.*;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealSlapdown implements SlapdownIO, AutoCloseable {
  public static TalonFX extensionMotor;

  /** Configures the motors */
  public RealSlapdown() {
    extensionMotor = new TalonFX(EXTENSION_PORT);

    // Rollers - Kraken X44
    // Check how integrated encoder works

    TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();

    // TODO Define confi
    extensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionMotorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    TalonUtils.addMotor(extensionMotor);
    FaultLogger.register(extensionMotor);
  }

  @Override
  /** extending/retracting the intake */
  public void setArmVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }

  @Override
  /** getting position of the intake (extended or retracted) */
  public double extensionPosition() {
    return extensionMotor.getPosition().getValue().in(Radians);
  }

  @Override
  public void close() throws Exception {
    extensionMotor.close();
  }
}
