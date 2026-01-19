package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.Ports;
import org.sciborgs1155.robot.intake.IntakeConstants.*;

public class RealIntake implements IntakeIO, AutoCloseable {
  public static TalonFX rightMotor;
  public static TalonFX extensionMotor;

  /** Configures the motors */
  public RealIntake() {
    rightMotor = new TalonFX(Ports.Intake.RIGHT_MOTOR_PORT);
    extensionMotor = new TalonFX(Ports.Intake.EXTENSION_PORT);

    // Rollers - Kraken X44
    // Check how integrated encoder works

    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();

    // TODO Define confi
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT.in(Amps);

    rightMotor.getConfigurator().apply(rightMotorConfig);

    extensionMotor.getConfigurator().apply(rightMotorConfig);

    TalonUtils.addMotor(rightMotor);
    TalonUtils.addMotor(extensionMotor);

    FaultLogger.register(rightMotor);
    FaultLogger.register(extensionMotor);
  }

  @Override
  /** velocity of the motor running the intake */
  public AngularVelocity rollerVelocity() {
    return rightMotor.getVelocity().getValue();
  }

  @Override
  /** starting the rollers to intake */
  public void setRollerVoltage() {
    rightMotor.setVoltage(IntakeConstants.ROLLER_VOLTAGE);
  }

  @Override
  /** extending/retracting the intake */
  public void setArmVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }

  @Override
  /** getting position of the intake (extended or retracted) */
  public double extensionPosition() {
    return extensionMotor.getPosition().getValue().in(Radians);
  }

  @Override
  public void close() throws Exception {
    rightMotor.close();
    extensionMotor.close();
  }
}
