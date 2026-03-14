package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  /** True turret angle in radians (mechanism space). */
  public double trueAngleRad() {
    return simulation.getAngleRads();
  }

  @Override
  public double encoderA() {
    double turretRot = trueAngleRad() / (2.0 * Math.PI);
    double encoderRot = turretRot * (TURRET_GEARING / ENCODER_A_GEARING);

    return 1.0 - MathUtil.inputModulus(encoderRot, 0.0, 1.0);
  }

  @Override
  public double encoderB() {
    double turretRot = trueAngleRad() / (2.0 * Math.PI);
    double encoderRot = turretRot * (TURRET_GEARING / ENCODER_B_GEARING);

    return 1.0 - MathUtil.inputModulus(encoderRot, 0.0, 1.0);
  }

  @Override
  public void setVoltage(double voltage) {
    simulation.setInputVoltage(voltage);
    simulation.update(PERIOD.in(Seconds));
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
