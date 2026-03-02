package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.sciborgs1155.robot.Ports.Turret.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.TalonUtils;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Real hardware interface for the {@code Turret} subsystem. */
public class RealTurret implements TurretIO {
  /** Motor controller that operates a motor which is used to rotate the turret. */
  private final TalonFX hardware = new TalonFX(MOTOR);

  private final CANcoder encoderA = new CANcoder(ENCODER_A);
  private final CANcoder encoderB = new CANcoder(ENCODER_B);

  private double lastGoodPositionRad;
  private double failCount;

  private final EasyCRTConfig crtConfig =
      new EasyCRTConfig(() -> Rotations.of(encoderA()), () -> Rotations.of(encoderB()))
          .withEncoderRatios(
              (double) TURRET_GEARING / ENCODER_A_GEARING,
              (double) TURRET_GEARING / ENCODER_B_GEARING)
          .withMechanismRange(MIN_ANGLE, MAX_ANGLE)
          .withMatchTolerance(CRT_MATCH_TOLERANCE)
          .withAbsoluteEncoderInversions(true, true);

  private final EasyCRT solverCRT = new EasyCRT(crtConfig);

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
    double encoderRot = trueAngleRot() * ((double) TURRET_GEARING / ENCODER_A_GEARING);

    return MathUtil.inputModulus(encoderRot, 0.0, 1.0);
  }

  /**
   * Derives encoder values from the integrated motor encoder. Note that this will not save pass
   * resets, so it should only be used for testing.
   */
  public double encoderBDerived() {
    double encoderRot = trueAngleRot() * ((double) TURRET_GEARING / ENCODER_B_GEARING);

    return MathUtil.inputModulus(encoderRot, 0.0, 1.0);
  }

  @Override
  public double encoderA() {
    return 1 - encoderA.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public double encoderB() {
    return 1 - encoderB.getAbsolutePosition().getValueAsDouble();
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
              failCount = 0;
              return lastGoodPositionRad;
            })
        .orElseGet(
            () -> {
              failCount++;
              if (failCount % 10 == 0) {
                FaultLogger.report(
                    new Fault(
                        "Turret CRT failure: >10 consecutive failures",
                        "Unable to solve turret position with CRT, using stale position - fail count: "
                            + failCount,
                        FaultType.WARNING));
              }
              return lastGoodPositionRad;
            });
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
    SmartDashboard.putNumber("trueAngleRot", trueAngleRot());
    SmartDashboard.putNumber("encoderADerived", encoderADerived());
    SmartDashboard.putNumber("encoderBDerived", encoderBDerived());
  }
}
