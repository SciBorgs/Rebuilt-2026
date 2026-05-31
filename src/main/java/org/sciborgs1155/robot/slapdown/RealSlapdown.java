package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.DRIVE_CANIVORE;
import static org.sciborgs1155.robot.Constants.INTAKE_CANIVORE;
import static org.sciborgs1155.robot.Ports.Slapdown.ENCODER;
import static org.sciborgs1155.robot.Ports.Slapdown.EXTENSION;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.MIN_ANGLE;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealSlapdown implements SlapdownIO {
  private final TalonFX motor;
  private final CANcoder encoder;

  // private final AbsoluteEncoder encoder;

  /** Configures the motors */
  public RealSlapdown() {
    motor = new TalonFX(EXTENSION, INTAKE_CANIVORE);
    encoder = new CANcoder(ENCODER, DRIVE_CANIVORE);
    // Rollers - Kraken X44
    // Check how integrated encoder works

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    motorConfig.Feedback.FeedbackRemoteSensorID = ENCODER;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    encoderConfig.MagnetSensor.MagnetOffset = -0.893798828125;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;

    motor.getConfigurator().apply(motorConfig);
    encoder.getConfigurator().apply(encoderConfig);

    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
    FaultLogger.register(encoder);
  }

  // extending/retracting the intake
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  // getting position of the intake (extended or retracted)
  @Override
  public double position() {
    return encoder.getAbsolutePosition().getValue().in(Radians);
  }

  @Override
  public double current() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public void resetPosition() {
    // motor.setPosition(MIN_ANGLE);
    encoder.setPosition(MIN_ANGLE);
  }
}
