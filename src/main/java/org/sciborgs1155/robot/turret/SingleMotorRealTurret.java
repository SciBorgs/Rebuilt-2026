package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Constants.TURRET_CANIVORE;
import static org.sciborgs1155.robot.Ports.Turret.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** Real hardware interface for the {@code Turret} subsystem. */
public class SingleMotorRealTurret implements TurretIO {
  /** Motor controller that operates a motor which is used to rotate the turret. */
  private final TalonFX hardware = new TalonFX(MOTOR, TURRET_CANIVORE);

  private final CANcoder encoderA = new CANcoder(ENCODER_A);

  private double lastReading;

  private double position;

  /** Real hardware interface for the {@code Turret} subsystem. */
  public SingleMotorRealTurret() {
    // CONFIG
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

    hardware.getConfigurator().apply(configuration);

    // TALON UTILS
    TalonUtils.addMotor(hardware);

    // FAULT LOGGER
    FaultLogger.register(hardware);
    lastReading = encoderA();
  }

  private void update(double encoderPosition) {
    Rotation2d encoderDisplacement =
        Rotation2d.fromRotations(encoderPosition).minus(Rotation2d.fromRotations(lastReading));
    double displacement =
        encoderDisplacement.getRadians() * ENCODER_A_GEARING / (double) TURRET_GEARING;
    lastReading = encoderPosition;
    position += displacement;
  }

  @Override
  public double encoderA() {
    return encoderA.getAbsolutePosition().getValueAsDouble() + 0.5;
  }

  @Override
  public double encoderB() {
    return 0;
  }

  @Override
  public void setVoltage(double voltage) {
    hardware.setVoltage(voltage);
  }

  @Override
  public double position() {
    return position;
  }

  @Override
  public double velocity() {
    return hardware.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }

  @Override
  public void periodic() {
    update(encoderA());
  }
}
