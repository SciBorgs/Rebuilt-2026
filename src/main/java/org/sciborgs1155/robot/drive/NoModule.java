package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;

public class NoModule implements ModuleIO {
  @Override
  public String name() {
    return "NoModule";
  }

  @Override
  public void setDriveVoltage(double voltage) {}

  @Override
  public void setTurnVoltage(double voltage) {}

  @Override
  public double drivePosition() {
    return 0;
  }

  @Override
  public double driveVelocity() {
    return 0;
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.kZero;
  }

  @Override
  public SwerveModuleState state() {
    return new SwerveModuleState();
  }

  @Override
  public SwerveModulePosition position() {
    return new SwerveModulePosition();
  }

  @Override
  public SwerveModuleState desiredState() {
    return new SwerveModuleState();
  }

  @Override
  public void resetEncoders() {}

  @Override
  public void setDriveSetpoint(double velocity) {}

  @Override
  public void setTurnSetpoint(Rotation2d angle) {}

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {}

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {}

  @Override
  public double[][] moduleOdometryData() {
    return new double[0][0];
  }

  public SwerveModulePosition[] odometryData() {
    return new SwerveModulePosition[0];
  }

  public double[] timestamps() {
    return new double[0];
  }

  @Override
  public void close() {}
}
