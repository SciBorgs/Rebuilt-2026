package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Turret.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** Real hardware interface for the {@code Turret} subsystem. */
public class RealTurret implements TurretIO {
  /**
   * Motor controller that operates a {@code Kraken x60} motor which is used to rotate the turret.
   */
  private final TalonFX motor = new TalonFX(MOTOR, CAN_BUS);

  /** Constructs the hardware implementation of the turret motor. */
  public RealTurret() {
    // TALON UTILS
    TalonUtils.addMotor(motor);

    // FAULT LOGGER
    FaultLogger.register(motor);

    // CONFIG
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    motor.getConfigurator().apply(configuration);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Volts));
  }

  @Override
  public Angle position() {
    return motor.getPosition().getValue();
  }

  @Override
  public AngularVelocity velocity() {
    return motor.getVelocity().getValue();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
