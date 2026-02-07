package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.sciborgs1155.robot.Constants.TURRET_CANIVORE;
import static org.sciborgs1155.robot.Ports.Turret.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Real hardware interface for the {@code Turret} subsystem. */
public class RealTurret implements TurretIO {
  /** Motor controller that operates a motor which is used to rotate the turret. */
  private final TalonFX hardware = new TalonFX(MOTOR, TURRET_CANIVORE);

  private final CANcoder encoderA = new CANcoder(ENCODER_A);
  private final CANcoder encoderB = new CANcoder(ENCODER_B);

  private double lastGoodPositionRad;

  private final EasyCRTConfig crtConfig =
      new EasyCRTConfig(() -> Rotations.of(encoderA()), () -> Rotations.of(encoderB()))
          .withEncoderRatios(
              (double) TURRET_GEARING / ENCODER_A_GEARING,
              (double) TURRET_GEARING / ENCODER_B_GEARING)
          .withMechanismRange(MIN_ANGLE, MAX_ANGLE)
          .withMatchTolerance(CRT_MATCH_TOLERANCE);

  private final EasyCRT solverCRT = new EasyCRT(crtConfig);

  /** Real hardware interface for the {@code Turret} subsystem. */
  public RealTurret() {
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
  }

  @Override
  public double encoderA() {
    return (encoderA.getAbsolutePosition().getValueAsDouble() + 1) / 2;
  }

  @Override
  public double encoderB() {
    return (encoderB.getAbsolutePosition().getValueAsDouble() + 1) / 2;
  }

  @Override
  public void setVoltage(double voltage) {
    hardware.setVoltage(voltage);
  }

  @Override
  public double position() {
    return solverCRT
        .getAngleOptional()
        .map(
            a -> {
              lastGoodPositionRad = a.in(Radians);
              return lastGoodPositionRad;
            })
        .orElse(lastGoodPositionRad);
  }

  @Override
  public double velocity() {
    return hardware.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
