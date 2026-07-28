package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Constants.SHOOTING_CANIVORE;
import static org.sciborgs1155.robot.Ports.Indexer.MOTOR;
import static org.sciborgs1155.robot.indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealIndexer implements IndexerIO {
  private final TalonFX motor;

  /** Creates a IndexerIO implementations with a real motor */
  public RealIndexer() {
    motor = new TalonFX(MOTOR, SHOOTING_CANIVORE);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = GEARING;

    motor.getConfigurator().apply(config);

    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
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
