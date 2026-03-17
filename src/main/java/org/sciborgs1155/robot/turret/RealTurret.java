package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.SHOOTING_CANIVORE;
import static org.sciborgs1155.robot.Ports.Turret.ENCODER_A;
import static org.sciborgs1155.robot.Ports.Turret.ENCODER_B;
import static org.sciborgs1155.robot.Ports.Turret.MOTOR;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** Real hardware interface for the {@code Turret} subsystem. */
public class RealTurret implements TurretIO {
  /** Motor controller that operates a motor which is used to rotate the turret. */
  private final TalonFX hardware = new TalonFX(MOTOR, SHOOTING_CANIVORE);

  private final CANcoder encoderA = new CANcoder(ENCODER_A, SHOOTING_CANIVORE);
  private final CANcoder encoderB = new CANcoder(ENCODER_B, SHOOTING_CANIVORE);

  /** Real hardware interface for the {@code Turret} subsystem. */
  public RealTurret() {
    // CONFIG
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    hardware.getConfigurator().apply(configuration);
    hardware.setPosition(0);

    final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    encoderA.getConfigurator().apply(encoderConfig);
    encoderB.getConfigurator().apply(encoderConfig);

    encoderA.setPosition(0);
    encoderB.setPosition(0);
    

    // TALON UTILS
    TalonUtils.addMotor(hardware);

    // FAULT LOGGER
    FaultLogger.register(hardware);
    FaultLogger.register(encoderA);
    FaultLogger.register(encoderB);
  }

  /**
   * Uses integrated motor encoder to get what encoder values should be (for testing, note that it
   * will not save pass resets)
   */
  public double trueAngleRot() {
    return hardware.getPosition().getValueAsDouble();
  }

  /**
   * Derives encoder values from the integrated motor encoder. Note that this will not save pass
   * resets, so it should only be used for testing.
   */
  public double encoderADerived() {
    double encoderRot = trueAngleRot() * (TURRET_GEARING / ENCODER_A_GEARING);

    return MathUtil.inputModulus(encoderRot, 0.0, 1.0);
  }

  /**
   * Derives encoder values from the integrated motor encoder. Note that this will not save pass
   * resets, so it should only be used for testing.
   */
  public double encoderBDerived() {
    double encoderRot = trueAngleRot() * (TURRET_GEARING / ENCODER_B_GEARING);

    return MathUtil.inputModulus(encoderRot, 0.0, 1.0);
  }

  @Override
  public double encoderA() {
    return encoderA.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public double encoderB() {
    return encoderB.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    hardware.setVoltage(voltage);
  }

  @Override
  public double voltage() {
    return hardware.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double velocity() {
    return hardware.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("trueAngleRot", trueAngleRot());
    SmartDashboard.putNumber("encoderADerived", encoderADerived());
    SmartDashboard.putNumber("encoderBDerived", encoderBDerived());
  }

  @Override
  public void setPosition(double pos) {
    hardware.setPosition(pos);
  }
}
