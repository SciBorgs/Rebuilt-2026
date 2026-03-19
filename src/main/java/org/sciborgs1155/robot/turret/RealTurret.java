package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.SHOOTING_CANIVORE;
import static org.sciborgs1155.robot.Ports.Turret.MOTOR;
import static org.sciborgs1155.robot.turret.TurretConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.turret.TurretConstants.GEAR_RATIO;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** Real hardware interface for the {@code Turret} subsystem. */
public class RealTurret implements TurretIO {
  /** Motor controller that operates a motor which is used to rotate the turret. */
  private final TalonFX hardware = new TalonFX(MOTOR, SHOOTING_CANIVORE);

  /** Real hardware interface for the {@code Turret} subsystem. */
  public RealTurret() {
    // CONFIG
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    hardware.getConfigurator().apply(configuration);
    hardware.setPosition(0);

    // TALON UTILS
    TalonUtils.addMotor(hardware);

    // FAULT LOGGER
    FaultLogger.register(hardware);
  }

  /**
   * Uses integrated motor encoder to get what encoder values should be (for testing, note that it
   * will not save pass resets)
   */
  @Override
  public double angle() {
    return hardware.getPosition().getValueAsDouble();
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
    SmartDashboard.putNumber("trueAngleRot", angle());
  }
}
