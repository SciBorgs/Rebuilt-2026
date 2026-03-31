package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static org.sciborgs1155.robot.Constants.SHOOTING_CANIVORE;
import static org.sciborgs1155.robot.Ports.Turret.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** Real hardware interface for the {@code Turret} subsystem. */
public class RealTurret implements TurretIO {
  /** Motor controller that operates a motor which is used to rotate the turret. */
  private final TalonFX motor = new TalonFX(MOTOR, SHOOTING_CANIVORE);

  private final CANcoder encoderA = new CANcoder(ENCODER_A, "shooting");
  private final CANcoder encoderB = new CANcoder(ENCODER_B, "shooting");

  /** Real hardware interface for the {@code Turret} subsystem. */
  public RealTurret() {
    // CONFIG
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(configuration);

    final CANcoderConfiguration encoderAConfig = new CANcoderConfiguration();
    encoderAConfig.MagnetSensor.MagnetOffset = -0.461;
    encoderAConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    final CANcoderConfiguration encoderBConfig = new CANcoderConfiguration();
    encoderBConfig.MagnetSensor.MagnetOffset = -0.862;
    encoderBConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderA.getConfigurator().apply(encoderAConfig);
    encoderB.getConfigurator().apply(encoderBConfig);

    // TALON UTILS
    TalonUtils.addMotor(motor);

    // FAULT LOGGER
    FaultLogger.register(motor);
  }

  @Override
  public double encoderA() {
    return encoderA.getAbsolutePosition().getValue().in(Rotations);
  }

  @Override
  public double encoderB() {
    return encoderB.getAbsolutePosition().getValue().in(Rotations);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double voltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("relative angle rot", motor.getPosition().getValue().in(Rotations));
    SmartDashboard.putNumber("encoder a", encoderA());
    SmartDashboard.putNumber("encoder b", encoderB());
  }

  @Override
  public void setPosition(Angle angle) {
    motor.setPosition(angle);
  }

  @Override
  public Angle getPosition() {
    return motor.getPosition().getValue();
  }
}
