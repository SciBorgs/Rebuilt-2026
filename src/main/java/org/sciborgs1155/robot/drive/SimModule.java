package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SimModule implements ModuleIO {
  private final DCMotorSim drive =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              Driving.FRONT_LEFT_FF.kV(), Driving.FRONT_LEFT_FF.kA()),
          DCMotor.getKrakenX60(1));

  private final PIDController driveFeedback =
      new PIDController(Driving.PID.P, Driving.PID.I, Driving.PID.D);

  private final SimpleMotorFeedforward driveFF =
      new SimpleMotorFeedforward(
          Driving.FRONT_LEFT_FF.kS(), Driving.FRONT_LEFT_FF.kV(), Driving.FRONT_LEFT_FF.kA());

  private final DCMotorSim turn =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Turning.FF.V, Turning.FF.A), DCMotor.getNeo550(1));

  private final PIDController turnFeedback =
      new PIDController(Turning.PID.P, Turning.PID.I, Turning.PID.D);

  private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  public SimModule(String name) {
    this.name = name;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    drive.setInputVoltage(voltage);
    drive.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turn.setInputVoltage(voltage);
    turn.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double drivePosition() {
    return drive.getAngularPositionRad();
  }

  @Override
  public double driveVelocity() {
    return drive.getAngularVelocityRadPerSec();
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turn.getAngularPositionRad());
  }

  @Override
  public SwerveModuleState state() {
    return new SwerveModuleState(driveVelocity(), rotation());
  }

  @Override
  public SwerveModulePosition position() {
    return new SwerveModulePosition(drivePosition(), rotation());
  }

  @Override
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    drive.setState(VecBuilder.fill(0, 0));
    turn.setState(VecBuilder.fill(0, 0));
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    setDriveVoltage(
        driveFeedback.calculate(driveVelocity(), velocity) + driveFF.calculate(velocity));
  }

  @Override
  public void setTurnSetpoint(Rotation2d setpoint) {
    setTurnVoltage(turnFeedback.calculate(rotation().getRadians(), setpoint.getRadians()));
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint.optimize(rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.cosineScale(rotation());

    if (mode == ControlMode.OPEN_LOOP_VELOCITY) {
      setDriveVoltage(driveFF.calculate(setpoint.speedMetersPerSecond));
    } else {
      setDriveSetpoint(setpoint.speedMetersPerSecond);
    }

    setTurnSetpoint(setpoint.angle);
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;

    double turnVolts = turnFeedback.calculate(rotation().getRadians(), setpoint.angle.getRadians());

    setDriveVoltage(voltage);
    setTurnVoltage(turnVolts);
  }

  @Override
  public double[][] moduleOdometryData() {
    return new double[][] {};
  }

  public SwerveModulePosition[] odometryData() {
    return new SwerveModulePosition[] {};
  }

  public double[] timestamps() {
    return new double[] {};
  }

  @Override
  public void close() {}
}
