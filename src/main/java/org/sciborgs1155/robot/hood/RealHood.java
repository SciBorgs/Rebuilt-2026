package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Hood.MOTOR_PORT;
import static org.sciborgs1155.robot.hood.HoodConstants.GEARING;
import static org.sciborgs1155.robot.hood.HoodConstants.STATOR_LIMIT;
import static org.sciborgs1155.robot.hood.HoodConstants.SUPPLY_LIMIT;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** Hood class with a motor controller */
public class RealHood implements HoodIO {

  private final TalonFX motor;
  private final TalonFXConfiguration config;

  /** constructor for real hood */
  public RealHood() {
    motor = new TalonFX(MOTOR_PORT);

    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEARING;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.setPosition(MIN_ANGLE);
    motor.getConfigurator().apply(config);

    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
  }

  /** gets the hood angle in rads */
  @Override
  public double angle() {
    return motor.getPosition().getValue().in(Radians);
  }

  /** sets the voltage of the hood motor */
  @Override
  public void setVoltage(double v) {
    motor.setVoltage(v);
  }

  /** rotational velocity of the hood in rads/sec */
  @Override
  public double velocity() {
    return motor.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public double getVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }
}
