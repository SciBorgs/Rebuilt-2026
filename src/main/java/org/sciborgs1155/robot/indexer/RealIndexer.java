package org.sciborgs1155.robot.indexer;

import static org.sciborgs1155.robot.Constants.INTAKE_CANIVORE;
import static org.sciborgs1155.robot.Ports.Indexer.MOTOR;
import static org.sciborgs1155.robot.indexer.IndexerConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;

public class RealIndexer implements IndexerIO{
    private final TalonFX motor;

    public RealIndexer() {
        motor = new TalonFX(MOTOR, INTAKE_CANIVORE);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
