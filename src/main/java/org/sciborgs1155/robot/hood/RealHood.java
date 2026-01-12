package org.sciborgs1155.robot.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Hood.*;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

public class RealHood implements HoodIO{

    private final TalonFX motor;
    private final TalonFXConfiguration config;

    public RealHood() {
        motor = new TalonFX(MOTOR_PORT);

        config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = CANCODER_GEARING;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.FeedbackRemoteSensorID = CANCODER;

        motor.getConfigurator().apply(config);

    }

    @Override
    public Angle angle() {
        return motor.getPosition().getValue();
    }

    @Override
    public void setVoltage(double v) {
        motor.setVoltage(v);
    }

    
    
}
