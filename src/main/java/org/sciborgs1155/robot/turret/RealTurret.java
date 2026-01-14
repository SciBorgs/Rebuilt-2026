package org.sciborgs1155.robot.turret;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static org.sciborgs1155.robot.turret.TurretConstants.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Turret.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class RealTurret implements TurretIO { // x44 for y and x60 for x
  public final TalonFX x = new TalonFX(X_MOTOR, CAN_BUS);
  public final TalonFX y = new TalonFX(Y_MOTOR, CAN_BUS);

  public RealTurret() {
    // CONFIG
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    // CONFIG
    x.getConfigurator().apply(configuration);
    y.getConfigurator().apply(configuration);

    // TALON UTILS
    TalonUtils.addMotor(x);
    TalonUtils.addMotor(y);

    // FAULT LOGGER
    FaultLogger.register(x);
    FaultLogger.register(y);
  }

  @Override
  public void setXVoltage(Voltage voltage) {
    x.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setYVoltage(Voltage voltage) {
    y.setVoltage(voltage.in(Volts));
  }

  @Override
  public Angle getXAngle() {
    return x.getPosition().getValue();
  }

  @Override
  public Angle getYAngle() {
    return y.getPosition().getValue();
  }
}
