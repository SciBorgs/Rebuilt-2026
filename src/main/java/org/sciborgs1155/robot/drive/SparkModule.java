package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.lib.LoggingUtils.log;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Queue;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.FFConstants;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SparkModule implements ModuleIO {
  private final SparkFlex driveMotor; // NEO Vortex
  private final SparkFlexConfig driveMotorConfig;
  private final SparkMax turnMotor; // NEO 550
  private final SparkMaxConfig turnMotorConfig;

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController turnPID;

  private final OdometryThread odometryThread;
  private final Queue<Double> position;
  private final Queue<Double> rotation;
  private final Queue<Double> timestamp;

  private final SimpleMotorFeedforward driveFF;

  private final Rotation2d angularOffset;

  private double lastPosition;
  private double lastVelocity;
  private Rotation2d lastRotation;

  @Logged private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  /**
   * Creates a new SparkModule with the specified configuration.
   *
   * @param drivePort The CAN ID of the drive motor.
   * @param turnPort The CAN ID of the turn motor.
   * @param angularOffset The angular offset of the module's encoder.
   * @param ff The feedforward constants for the drive motor.
   * @param name The name of the module.
   * @param invert Whether to invert the encoder direction.
   */
  public SparkModule(
      int drivePort,
      int turnPort,
      Rotation2d angularOffset,
      FFConstants ff,
      String name,
      boolean invert) {
    // Drive Motor

    driveMotor = new SparkFlex(drivePort, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getClosedLoopController();
    driveFF = new SimpleMotorFeedforward(ff.kS(), ff.kV(), ff.kA());
    driveMotorConfig = new SparkFlexConfig();

    check(
        driveMotor,
        driveMotor.configure(
            driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    driveMotorConfig.apply(
        driveMotorConfig
            .closedLoop
            .pid(Driving.PID.P, Driving.PID.I, Driving.PID.D)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder));

    driveMotorConfig.apply(
        driveMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)));

    driveMotorConfig.apply(
        driveMotorConfig
            .encoder
            .positionConversionFactor(Driving.POSITION_FACTOR.in(Meters))
            .velocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond))
            .uvwAverageDepth(16)
            .uvwMeasurementPeriod(32));

    driveMotorConfig.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));

    check(
        driveMotor,
        driveMotor.configure(
            driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    // Turn Motor

    turnMotor = new SparkMax(turnPort, MotorType.kBrushless);
    turningEncoder = turnMotor.getAbsoluteEncoder();
    turnPID = turnMotor.getClosedLoopController();
    turnMotorConfig = new SparkMaxConfig();

    check(
        turnMotor,
        turnMotor.configure(
            turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    turnMotorConfig.apply(
        turnMotorConfig
            .closedLoop
            .pid(Turning.PID.P, Turning.PID.I, Turning.PID.D)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder));

    turnMotorConfig.apply(
        turnMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));

    turnMotorConfig.apply(turnMotorConfig.absoluteEncoder.inverted(invert));

    turnMotorConfig.apply(
        turnMotorConfig
            .absoluteEncoder
            .positionConversionFactor(Turning.POSITION_FACTOR.in(Radians))
            .velocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond))
            .averageDepth(2));

    turnMotorConfig.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.ABSOLUTE),
            false));

    check(
        turnMotor,
        turnMotor.configure(
            turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    register(driveMotor);
    register(turnMotor);

    odometryThread = OdometryThread.getInstance();

    position = odometryThread.registerSignal(() -> driveEncoder.getPosition());
    rotation = odometryThread.registerSignal(() -> turningEncoder.getPosition());

    timestamp = odometryThread.makeTimestampQueue();

    resetEncoders();

    this.angularOffset = angularOffset;
    this.name = name;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
    check(driveMotor);
    log("/Robot/drive/" + name + "/drive current", driveMotor.getOutputCurrent());
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
    check(turnMotor);
    log("/Robot/drive/" + name + "/turn current", turnMotor.getOutputCurrent());
  }

  @Override
  public double drivePosition() {
    lastPosition = SparkUtils.wrapCall(driveMotor, driveEncoder.getPosition()).orElse(lastPosition);
    // account for rotation of turn motor on rotation of drive motor
    return lastPosition - turningEncoder.getPosition() * COUPLING_RATIO;
  }

  @Override
  public double driveVelocity() {
    lastVelocity = SparkUtils.wrapCall(driveMotor, driveEncoder.getVelocity()).orElse(lastVelocity);
    return lastVelocity;
  }

  @Override
  public Rotation2d rotation() {
    lastRotation =
        SparkUtils.wrapCall(
                turnMotor,
                Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset))
            .orElse(lastRotation);
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
    driveEncoder.setPosition(0);
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    drivePID.setSetpoint(
        velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFF.calculate(velocity));
  }

  @Override
  public void setTurnSetpoint(Rotation2d angle) {
    turnPID.setSetpoint(angle.getRadians(), ControlType.kPosition);
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
    Drive.LOCK.lock();
    try {
      return new double[][] {
        position.stream().mapToDouble((Double d) -> d).toArray(),
        rotation.stream().mapToDouble((Double d) -> d).toArray(),
        timestamp.stream().mapToDouble((Double d) -> d).toArray()
      };
    } finally {
      Drive.LOCK.unlock();
    }
  }

  @Override
  public SwerveModulePosition[] odometryData() {
    SwerveModulePosition[] positions = new SwerveModulePosition[20];
    Drive.LOCK.lock();

    var data = moduleOdometryData();

    for (int i = 0; i < data[0].length; i++) {
      positions[i] = new SwerveModulePosition(data[0][i], Rotation2d.fromRotations(data[1][i]));
    }

    position.clear();
    rotation.clear();
    timestamp.clear();

    Drive.LOCK.unlock();
    return positions;
  }

  @Override
  public double[] timestamps() {
    return moduleOdometryData()[2];
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
