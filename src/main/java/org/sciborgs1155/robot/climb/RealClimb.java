package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Climb.*;
import static org.sciborgs1155.robot.climb.ClimbConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealClimb implements ClimbIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  /** A constuctor for a real climb object. */
  public RealClimb() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftMotor = new TalonFX(LEFT_MOTOR_ID);
    rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    rightMotor.setControl(new Follower(LEFT_MOTOR_ID, MotorAlignmentValue.Aligned));

    leftConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    leftConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.Feedback.SensorToMechanismRatio = GEARING;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
    rightConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.Feedback.SensorToMechanismRatio = GEARING;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    FaultLogger.register(leftMotor);
    FaultLogger.register(rightMotor);
    TalonUtils.addMotor(leftMotor);
    TalonUtils.addMotor(rightMotor);
  }

  @Override
  public double position() {
    return leftMotor.getPosition().getValueAsDouble() * SENSOR_TO_MECHANISM_RATIO;
  }

  @Override
  public void setVoltage(double volt) {
    leftMotor.setVoltage(volt);
  }

  @Override
  public double velocity() {
    return leftMotor.getVelocity().getValueAsDouble() * SENSOR_TO_MECHANISM_RATIO;
  }

  @Override
  public void close() {
    leftMotor.close();
  }
}
