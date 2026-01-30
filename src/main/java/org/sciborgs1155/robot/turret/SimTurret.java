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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
          TURRET_LENGTH.in(Meters), // ARM LENGTH
          MIN_ANGLE.in(Radians), // MINIMUM ANGLE
          MAX_ANGLE.in(Radians), // MAXIMUM ANGLE
          false, // GRAVITY DISBLAED
          START_ANGLE.in(Radians)); // STARTING ANGLE

  /** Cached last valid CRT solution */
  private double lastGoodPositionRad = START_ANGLE.in(Radians);

  /** EasyCRT solver */
  private final EasyCRTConfig crtConfig =
      new EasyCRTConfig(this::encoderARotations, this::encoderBRotations)
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

  /** Absolute encoder A output as Angle [0,1) rotations. */
  private Angle encoderARotations() {
    double turretRot = trueAngleRad() / (2.0 * Math.PI);
    double encoderRot = turretRot * ((double) TURRET_GEARING / ENCODER_A_GEARING);

    return Rotations.of(MathUtil.inputModulus(encoderRot, 0.0, 1.0));
  }

  /** Absolute encoder B output as Angle [0,1) rotations. */
  private Angle encoderBRotations() {
    double turretRot = trueAngleRad() / (2.0 * Math.PI);
    double encoderRot = turretRot * ((double) TURRET_GEARING / ENCODER_B_GEARING);

    return Rotations.of(MathUtil.inputModulus(encoderRot, 0.0, 1.0));
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
              return lastGoodPositionRad;
            })
        .orElse(lastGoodPositionRad);
  }

  @Override
  public double velocity() {
    return simulation.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
