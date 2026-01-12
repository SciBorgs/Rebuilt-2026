package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.Constants.DRIVE_CANIVORE;
import static org.sciborgs1155.robot.Constants.ODOMETRY_PERIOD;
import static org.sciborgs1155.robot.Constants.PERIOD;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Queue;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.FFConstants;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor; // Kraken X60
  private final TalonFX turnMotor; // Kraken X60
  private final CANcoder encoder;

  private final VelocityVoltage velocityOut = new VelocityVoltage(0);
  private final PositionVoltage rotationsIn = new PositionVoltage(0);

  private final OdometryThread talonThread;
  private final Queue<Double> position;
  private final Queue<Double> rotation;
  private final Queue<Double> timestamp;

  private final SimpleMotorFeedforward driveFF;

  @Logged private SwerveModuleState setpoint = new SwerveModuleState();

  private Rotation2d lastRotation;

  private final String name;

  public TalonModule(
      int drivePort,
      int turnPort,
      int sensorID,
      Rotation2d angularOffset,
      FFConstants ff,
      String name,
      boolean invert) {
    // drive motor
    driveMotor = new TalonFX(drivePort, DRIVE_CANIVORE);
    driveFF = new SimpleMotorFeedforward(ff.kS(), ff.kV(), ff.kA());

    TalonFXConfiguration talonDriveConfig = new TalonFXConfiguration();

    talonDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonDriveConfig.Feedback.SensorToMechanismRatio =
        Driving.GEARING / Driving.CIRCUMFERENCE.in(Meters);
    talonDriveConfig.CurrentLimits.StatorCurrentLimit = Driving.STATOR_LIMIT.in(Amps);

    talonDriveConfig.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    talonDriveConfig.Slot0.kP = Driving.PID.P;
    talonDriveConfig.Slot0.kI = Driving.PID.I;
    talonDriveConfig.Slot0.kD = Driving.PID.D;

    turnMotor = new TalonFX(turnPort, DRIVE_CANIVORE);
    encoder = new CANcoder(sensorID, DRIVE_CANIVORE);

    // turn motor
    TalonFXConfiguration talonTurnConfig = new TalonFXConfiguration();

    talonTurnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonTurnConfig.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    talonTurnConfig.Feedback.SensorToMechanismRatio = Turning.ENCODER_GEARING;
    talonTurnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonTurnConfig.Feedback.FeedbackRemoteSensorID = sensorID;

    talonTurnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    talonTurnConfig.Slot0.kP = Turning.PID.P;
    talonTurnConfig.Slot0.kI = Turning.PID.I;
    talonTurnConfig.Slot0.kD = Turning.PID.D;

    talonTurnConfig.CurrentLimits.StatorCurrentLimit = Turning.CURRENT_LIMIT.in(Amps);

    for (int i = 0; i < 5; i++) {
      StatusCode success = driveMotor.getConfigurator().apply(talonDriveConfig);
      if (success.isOK()) break;
    }

    for (int i = 0; i < 5; i++) {
      StatusCode success = turnMotor.getConfigurator().apply(talonTurnConfig);
      if (success.isOK()) break;
    }

    // reduces update frequency on unnecessary signals
    // only reset on robot restart and redeploy or calling motor.resetSignalFrequencies()
    ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / ODOMETRY_PERIOD.in(Seconds),
        driveMotor.getPosition(),
        driveMotor.getVelocity(),
        turnMotor.getPosition(),
        turnMotor.getVelocity());

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / PERIOD.in(Seconds), driveMotor.getMotorVoltage(), turnMotor.getMotorVoltage());

    register(driveMotor);
    register(turnMotor);
    register(encoder);

    TalonUtils.addMotor(driveMotor);
    TalonUtils.addMotor(turnMotor);

    talonThread = OdometryThread.getInstance();

    position = talonThread.registerSignal(driveMotor.getPosition());
    rotation = talonThread.registerSignal(turnMotor.getPosition());

    timestamp = talonThread.makeTimestampQueue();

    resetEncoders();

    this.name = name;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public double drivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double driveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public Rotation2d rotation() {
    lastRotation = Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble());
    return lastRotation;
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
    driveMotor.setPosition(0);
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    driveMotor.setControl(
        velocityOut.withVelocity(velocity).withFeedForward(driveFF.calculate(velocity)));
  }

  @Override
  public void setTurnSetpoint(Rotation2d angle) {
    turnMotor.setControl(rotationsIn.withPosition(angle.getRotations()).withSlot(0));
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    Rotation2d rotation = rotation();
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint.optimize(rotation);
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.cosineScale(rotation);

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
    setDriveVoltage(voltage);
    setTurnSetpoint(angle);
  }

  @Override
  public double[][] moduleOdometryData() {
    Drive.lock.lock();
    try {
      double[][] data = {
        position.stream().mapToDouble((Double d) -> d).toArray(),
        rotation.stream().mapToDouble((Double d) -> d).toArray(),
        timestamp.stream().mapToDouble((Double d) -> d).toArray()
      };
      return data;
    } finally {
      Drive.lock.unlock();
    }
  }

  public SwerveModulePosition[] odometryData() {
    SwerveModulePosition[] positions = new SwerveModulePosition[20];
    Drive.lock.lock();

    var data = moduleOdometryData();

    for (int i = 0; i < data[0].length; i++) {
      positions[i] = new SwerveModulePosition(data[0][i], Rotation2d.fromRotations(data[1][i]));
    }

    position.clear();
    rotation.clear();
    timestamp.clear();

    Drive.lock.unlock();
    return positions;
  }

  public double[] timestamps() {
    return moduleOdometryData()[2];
  }

  @Override
  public void close() {
    turnMotor.close();
    driveMotor.close();
  }
}
