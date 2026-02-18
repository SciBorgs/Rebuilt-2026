package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Slapdown.*;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.GEARING;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealSlapdown implements SlapdownIO {
  private final TalonFX motor;

  /** Configures the motors */
  public RealSlapdown() {
    motor = new TalonFX(EXTENSION);

    // Rollers - Kraken X44
    // Check how integrated encoder works

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    motorConfig.Feedback.SensorToMechanismRatio = GEARING;

    motor.getConfigurator().apply(motorConfig);
    motor.setPosition(MAX_ANGLE);

    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
  }

  // extending/retracting the intake
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  // getting position of the intake (extended or retracted)
  @Override
  public double position() {
    return motor.getPosition().getValue().in(Radians);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
