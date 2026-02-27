package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Simulated hardware interface for the {@code Turret} subsystem. */
public class SimTurret implements TurretIO {
  /** Simulated servo motor representing the turret. */
  private final SingleJointedArmSim simulation =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), // GEARBOX
          GEAR_RATIO, // GEARING
          MOI.in(KilogramSquareMeters), // MOMENT OF INERTIA
          TURRET_RADIUS.in(Meters), // TURRET RADIUS
          MIN_ANGLE.in(Radians), // MINIMUM ANGLE
          MAX_ANGLE.in(Radians), // MAXIMUM ANGLE
          false, // GRAVITY DISBLAED
          START_ANGLE.in(Radians)); // STARTING ANGLE

  /** Cached last valid CRT solution */
  private double lastGoodPositionRad = START_ANGLE.in(Radians);

  private static final double DEVIATION = .05; // rotations
  private double failCount = 0;

  /** EasyCRT solver */
  private final EasyCRTConfig crtConfig =
      new EasyCRTConfig(() -> Rotations.of(encoderA()), () -> Rotations.of(encoderB()))
          .withEncoderRatios(
              (double) TURRET_GEARING / ENCODER_A_GEARING,
              (double) TURRET_GEARING / ENCODER_B_GEARING)
          .withMechanismRange(MIN_ANGLE, MAX_ANGLE)
          .withMatchTolerance(CRT_MATCH_TOLERANCE);

  private final EasyCRT solverCRT = new EasyCRT(crtConfig);

  /** True turret angle in radians (mechanism space). */
  public double trueAngleRad() {
    return simulation.getAngleRads();
  }

  @Override
  public double encoderA() {
    double turretRot = trueAngleRad() / (2.0 * Math.PI);
    double encoderRot = turretRot * ((double) TURRET_GEARING / ENCODER_A_GEARING);

    return MathUtil.inputModulus(encoderRot + Math.random() * DEVIATION, 0.0, 1.0);
  }

  @Override
  public double encoderB() {
    double turretRot = trueAngleRad() / (2.0 * Math.PI);
    double encoderRot = turretRot * ((double) TURRET_GEARING / ENCODER_B_GEARING);

    return MathUtil.inputModulus(encoderRot + Math.random() * DEVIATION, 0.0, 1.0);
  }

  @Override
  public void setVoltage(double voltage) {
    simulation.setInputVoltage(voltage);
    simulation.update(PERIOD.in(Seconds));
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
    return simulation.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("trueAngle", trueAngleRad());
  }
}
