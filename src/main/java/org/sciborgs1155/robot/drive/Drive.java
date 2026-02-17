package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.atan;
import static org.sciborgs1155.lib.Assertion.*;
import static org.sciborgs1155.lib.LoggingUtils.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.Constants.allianceRotation;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Arrays;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Assertion.TruthAssertion;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.DriveConstants.Assisted;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Skid;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.vision.Vision.PoseEstimate;

@SuppressWarnings("PMD.GodClass")
public class Drive extends SubsystemBase implements AutoCloseable {
  // Modules
  private final ModuleIO frontLeft;
  private final ModuleIO frontRight;
  private final ModuleIO rearLeft;
  private final ModuleIO rearRight;

  private final List<ModuleIO> modules;

  private final BooleanSupplier[] modulesStalling;

  // Gyro, navX2-MXP
  private final GyroIO gyro;
  @Logged private static Rotation2d simRotation = Rotation2d.kZero;

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  // Translation PID Tuning
  @NotLogged
  private final DoubleEntry translationP =
      Tuning.entry("Robot/tuning/drive/translation p", Translation.P);

  @NotLogged
  private final DoubleEntry translationI =
      Tuning.entry("Robot/tuning/drive/translation i", Translation.I);

  @NotLogged
  private final DoubleEntry translationD =
      Tuning.entry("Robot/tuning/drive/translation d", Translation.D);

  // Rotation PID Tuning
  @NotLogged
  private final DoubleEntry rotationP = Tuning.entry("Robot/tuning/drive/rotation p", Rotation.P);

  @NotLogged
  private final DoubleEntry rotationI = Tuning.entry("Robot/tuning/drive/rotation i", Rotation.I);

  @NotLogged
  private final DoubleEntry rotationD = Tuning.entry("Robot/tuning/drive/rotation d", Rotation.D);

  // Max acceleration and skid
  @NotLogged
  private final DoubleEntry maxAccel =
      Tuning.entry("Robot/tuning/drive/Max Accel", MAX_ACCEL.in(MetersPerSecondPerSecond));

  @NotLogged private final DoubleEntry maxSkidAccel;

  @NotLogged
  private final DoubleEntry maxTiltAccel =
      Tuning.entry(
          "Robot/tuning/drive/Max Tilt Accel", MAX_TILT_ACCEL.in(MetersPerSecondPerSecond));

  // Odometry and pose estimation
  private final SwerveDrivePoseEstimator odometry;

  // Faster Odometry
  private SwerveModulePosition[] lastPositions;
  private Rotation2d lastHeading;
  public static final ReentrantLock LOCK = new ReentrantLock();

  @Logged private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d;

  // Characterization routines
  private final SysIdRoutine translationCharacterization;
  private final SysIdRoutine rotationalCharacterization;

  // Movement automation
  @Logged
  private final ProfiledPIDController translationController =
      new ProfiledPIDController(
          translationP.get(),
          translationI.get(),
          translationD.get(),
          new TrapezoidProfile.Constraints(
              MAX_SPEED.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  @Logged
  private final PIDController rotationController =
      new PIDController(rotationP.get(), rotationI.get(), rotationD.get());

  /**
   * A factory to create a new swerve drive based on the type of module used / real or simulation.
   */
  public static Drive create() {
    if (Robot.isReal()) {
      return switch (TYPE) {
        case TALON ->
            new Drive(
                new ReduxGyro(),
                new TalonModule(
                    FRONT_LEFT_DRIVE,
                    FRONT_LEFT_TURNING,
                    FRONT_LEFT_CANCODER,
                    ANGULAR_OFFSETS.get(0),
                    Driving.FF_CONSTANTS.get(0),
                    "FL",
                    true),
                new TalonModule(
                    FRONT_RIGHT_DRIVE,
                    FRONT_RIGHT_TURNING,
                    FRONT_RIGHT_CANCODER,
                    ANGULAR_OFFSETS.get(1),
                    Driving.FF_CONSTANTS.get(1),
                    "FR",
                    true),
                new TalonModule(
                    REAR_LEFT_DRIVE,
                    REAR_LEFT_TURNING,
                    REAR_LEFT_CANCODER,
                    ANGULAR_OFFSETS.get(2),
                    Driving.FF_CONSTANTS.get(2),
                    "RL",
                    true),
                new TalonModule(
                    REAR_RIGHT_DRIVE,
                    REAR_RIGHT_TURNING,
                    REAR_RIGHT_CANCODER,
                    ANGULAR_OFFSETS.get(3),
                    Driving.FF_CONSTANTS.get(3),
                    "RR",
                    true));
        case SPARK ->
            new Drive(
                new NavXGyro(),
                new SparkModule(
                    FRONT_LEFT_DRIVE,
                    FRONT_LEFT_TURNING,
                    ANGULAR_OFFSETS.get(0),
                    Driving.FF_CONSTANTS.get(0),
                    "FL",
                    true),
                new SparkModule(
                    FRONT_RIGHT_DRIVE,
                    FRONT_RIGHT_TURNING,
                    ANGULAR_OFFSETS.get(1),
                    Driving.FF_CONSTANTS.get(1),
                    "FR",
                    true),
                new SparkModule(
                    REAR_LEFT_DRIVE,
                    REAR_LEFT_TURNING,
                    ANGULAR_OFFSETS.get(2),
                    Driving.FF_CONSTANTS.get(2),
                    "RL",
                    true),
                new SparkModule(
                    REAR_RIGHT_DRIVE,
                    REAR_RIGHT_TURNING,
                    ANGULAR_OFFSETS.get(3),
                    Driving.FF_CONSTANTS.get(3),
                    "RR",
                    true));
      };
    } else {
      return new Drive(
          new NoGyro(),
          new SimModule("FL"),
          new SimModule("FR"),
          new SimModule("RL"),
          new SimModule("RR"));
    }
  }

  /** A factory to create a nonexistent swerve drive. */
  public static Drive none() {
    return new Drive(new NoGyro(), new NoModule(), new NoModule(), new NoModule(), new NoModule());
  }

  /** A swerve drive subsystem containing four {@link ModuleIO} modules and a gyroscope. */
  public Drive(
      GyroIO gyro, ModuleIO frontLeft, ModuleIO frontRight, ModuleIO rearLeft, ModuleIO rearRight) {
    // Swerve drive bits
    this.gyro = gyro;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    // Max skid accleration
    this.maxSkidAccel =
        Tuning.entry(
            "Robot/tuning/drive/Max Skid Accel", MAX_SKID_ACCEL.in(MetersPerSecondPerSecond));

    // Module lists
    modules = List.of(this.frontLeft, this.frontRight, this.rearLeft, this.rearRight);
    modules2d = new FieldObject2d[modules.size()];

    // Faster odometry
    lastPositions = modulePositions();
    lastHeading = gyro.rotation2d();

    // Stalling
    modulesStalling =
        modules.stream()
            .map(
                m ->
                    new Trigger(
                            () ->
                                Math.abs(m.desiredState().speedMetersPerSecond) > 0.3
                                    && Math.abs(m.state().speedMetersPerSecond) < 0.1)
                        .debounce(0.2, DebounceType.kRising)
                        .debounce(0.04, DebounceType.kFalling))
            .toArray(BooleanSupplier[]::new);

    // Characterization
    translationCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("translation state", state.toString())),
            new SysIdRoutine.Mechanism(
                volts ->
                    modules.forEach(
                        m -> m.updateInputs(Rotation2d.fromRadians(0), volts.in(Volts))),
                null,
                this,
                "translation"));
    rotationalCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("rotation state", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  this.frontLeft.updateInputs(
                      Rotation2d.fromRadians(3 * Math.PI / 4), volts.in(Volts));
                  this.frontRight.updateInputs(
                      Rotation2d.fromRadians(Math.PI / 4), volts.in(Volts));
                  this.rearLeft.updateInputs(
                      Rotation2d.fromRadians(-3 * Math.PI / 4), volts.in(Volts));
                  this.rearRight.updateInputs(
                      Rotation2d.fromRadians(-Math.PI / 4), volts.in(Volts));
                },
                null,
                this,
                "rotation"));

    gyro.reset(Rotation2d.kZero);
    odometry = new SwerveDrivePoseEstimator(kinematics, lastHeading, lastPositions, Pose2d.kZero);

    for (int i = 0; i < modules.size(); i++) {
      var module = modules.get(i);
      modules2d[i] = field2d.getObject("module-" + module.name());
    }

    translationController.setTolerance(Translation.TOLERANCE.in(Meters));
    rotationController.enableContinuousInput(0, 2 * Math.PI);
    rotationController.setTolerance(Rotation.TOLERANCE.in(Radians));

    OdometryThread.getInstance().start();

    if (TUNING) {
      SmartDashboard.putData(
          "Robot/translation/quasistatic forward",
          translationCharacterization
              .quasistatic(Direction.kForward)
              .withName("translation quasistatic forward"));
      SmartDashboard.putData(
          "Robot/translation/dynamic forward",
          translationCharacterization
              .dynamic(Direction.kForward)
              .withName("translation dynamic forward"));
      SmartDashboard.putData(
          "Robot/translation/quasistatic backward",
          translationCharacterization.quasistatic(Direction.kReverse));
      SmartDashboard.putData(
          "Robot/translation/dynamic backward",
          translationCharacterization
              .dynamic(Direction.kReverse)
              .withName("translation quasistatic backward"));
      SmartDashboard.putData(
          "Robot/rotation/quasistatic forward",
          rotationalCharacterization.quasistatic(Direction.kForward));
      SmartDashboard.putData(
          "Robot/rotation/dynamic forward",
          rotationalCharacterization
              .dynamic(Direction.kForward)
              .withName("rotation quasistatic forward"));
      SmartDashboard.putData(
          "Robot/rotation/quasistatic backward",
          rotationalCharacterization
              .quasistatic(Direction.kReverse)
              .withName("rotation quasistatic backward"));
      SmartDashboard.putData(
          "Robot/rotation/dynamic backward",
          rotationalCharacterization
              .dynamic(Direction.kReverse)
              .withName("rotation dynamic backward"));
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Logged
  public Pose2d pose() {
    return odometry.getEstimatedPosition();
  }

  /** Returns a Pose3D of the estimated pose of the robot. */
  public Pose3d pose3d() {
    return new Pose3d(odometry.getEstimatedPosition());
  }

  /**
   * Returns the currently-estimated field-relative yaw of the robot.
   *
   * @return The rotation.
   */
  @Logged
  public Rotation2d heading() {
    return pose().getRotation();
  }

  /**
   * Return the gyro's heading (unaffected by vision and wheel odometry throughout a match).
   *
   * <p>This value is reset to odometry heading on each robot enable. Primarily used for
   * field-relative applications.
   *
   * @return The gyro heading, set after enable to be field-relative after odometry correction.
   */
  @Logged
  public Rotation2d gyroHeading() {
    return lastHeading;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(lastHeading, lastPositions, pose);
  }

  /**
   * Drives the robot based on a {@link InputStream} for field relative x y and omega velocities.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @return The driving command.
   */
  public Command drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vOmega) {
    return run(
        () ->
            setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx.getAsDouble(),
                    vy.getAsDouble(),
                    vOmega.getAsDouble(),
                    heading().plus(allianceRotation())),
                ControlMode.OPEN_LOOP_VELOCITY));
  }

  /**
   * Drives the robot based on a {@link InputStream} for field relative x y and omega velocities.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param heading A supplier for the field relative heading of the robot.
   * @return The driving command.
   */
  public Command drive(DoubleSupplier vx, DoubleSupplier vy, Supplier<Rotation2d> heading) {
    return drive(
            vx,
            vy,
            () -> rotationController.calculate(heading().getRadians(), heading.get().getRadians()))
        .beforeStarting(rotationController::reset);
  }

  /**
   * Drives the robot based in a {@link InputStream} for field-relative x, y, and omega velocities.
   * Also adds a little extra translational velocity to move the robot to a certain desired position
   * if the driver is already attempting to move in that general direction. This command does not
   * assist in controlling the rotation of the robot.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @param target The target field-relative position for the robot, as a {@link Translation2d}.
   * @return The assisted driving command.
   */
  public Command assistedDrive(
      DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vOmega, Translation2d target) {
    return run(() -> {
          Vector<N2> driverVel = VecBuilder.fill(vx.getAsDouble(), vy.getAsDouble());
          Vector<N2> displacement = pose().getTranslation().minus(target).toVector();
          Vector<N2> perpDisplacement = displacement.projection(driverVel).minus(displacement);
          Vector<N2> result =
              driverVel.plus(
                  perpDisplacement
                      .unit()
                      .times(translationController.calculate(perpDisplacement.norm(), 0)));
          setChassisSpeeds(
              Math.acos(driverVel.unit().dot(displacement.unit()))
                          < Assisted.DRIVING_THRESHOLD.in(Radians)
                      && !Double.isNaN(result.norm())
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      result.get(0),
                      result.get(1),
                      vOmega.getAsDouble(),
                      heading().plus(allianceRotation()))
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      vx.getAsDouble(),
                      vy.getAsDouble(),
                      vOmega.getAsDouble(),
                      heading().plus(allianceRotation())),
              ControlMode.CLOSED_LOOP_VELOCITY);
        })
        .repeatedly();
  }

  /**
   * Drives the robot based in a {@link InputStream} for field relative x y and omega velocities.
   * Also adds a little translational velocity to move the robot to a certain desired position if
   * the driver is already attempting to move in that general direction. If the driver is not
   * currently attempting to rotate the robot, this command will also automatically rotate the robot
   * to a desired heading.
   *
   * @param vx A supplier for the velocity of the robot along the x axis (perpendicular to the
   *     alliance side).
   * @param vy A supplier for the velocity of the robot along the y axis (parallel to the alliance
   *     side).
   * @param vOmega A supplier for the angular velocity of the robot.
   * @param target The target field-relative position for the robot, as a {@link Pose2d}.
   * @return The assisted driving command.
   */
  public Command assistedDrive(
      DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vOmega, Pose2d target) {
    return assistedDrive(
            vx,
            vy,
            () ->
                Math.abs(target.getRotation().getRadians() - heading().getRadians())
                        > Rotation.TOLERANCE.in(Radians)
                    ? rotationController.calculate(
                        heading().minus(target.getRotation()).getRadians(), 0)
                    : vOmega.getAsDouble(),
            target.getTranslation())
        .until(() -> vOmega.getAsDouble() > Assisted.ROTATING_THRESHOLD)
        .andThen(assistedDrive(vx, vy, vOmega, target.getTranslation()));
  }

  /**
   * Drives the robot while facing a target pose.
   *
   * @param vx A supplier for the absolute x velocity of the robot.
   * @param vy A supplier for the absolute y velocity of the robot.
   * @param translation A supplier for the translation2d to face on the field.
   * @return A command to drive while facing a target.
   */
  public Command driveFacingTarget(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Translation2d> translation) {
    return drive(vx, vy, () -> translation.get().minus(pose().getTranslation()).getAngle());
  }

  /**
   * Checks whether the robot is at its rotational setpoint.
   *
   * @return Whether the rotation controller is at its setpoint.
   */
  @Logged
  public boolean atRotationalSetpoint() {
    return rotationController.atSetpoint();
  }

  /**
   * Checks whether the robot is facing towards a point on the field.
   *
   * @param target The field-relative point to check.
   * @return Whether the robot is facing the target closely enough.
   */
  public boolean isFacing(Translation2d target) {
    return Math.abs(
            gyro.rotation2d().getRadians()
                - target.minus(pose().getTranslation()).getAngle().getRadians())
        < rotationController.getErrorTolerance();
  }

  /**
   * Checks whether the robot is at a certain field coordinate.
   *
   * @param position The target position.
   * @param tolerance The tolerance, above which the robot will not be considered to be "at" the
   *     target.
   * @return Whether the robot is at a target translation.
   */
  public boolean atTranslation(Translation2d position, Distance tolerance) {
    return pose().getTranslation().minus(position).getNorm() < tolerance.in(Meters);
  }

  /**
   * Checks whether the robot is at a certain rotation.
   *
   * @param rotation The target rotation.
   * @param tolerance The tolerance, above which the robot will not be considered to be "at" the
   *     rotation.
   * @return Whether the robot is at a target rotation.
   */
  public boolean atRotation(Rotation2d rotation, Angle tolerance) {
    return Math.abs(heading().minus(rotation).getRadians()) < tolerance.in(Radians);
  }

  /**
   * Checks whether the robot is at a certain pose.
   *
   * @param pose The target pose.
   * @param translationTolerance The translational tolerance.
   * @param rotationTolerance The rotational tolerance.
   * @return Whether the robot is at a target pose.
   */
  public boolean atPose(Pose2d pose, Distance translationTolerance, Angle rotationTolerance) {
    return atTranslation(pose.getTranslation(), translationTolerance)
        && atRotation(pose.getRotation(), rotationTolerance);
  }

  /**
   * Checks whether the robot is at a certain pose. Uses the DriveConstants tolerances.
   *
   * @param pose The target pose.
   * @return Whether the robot is at a target pose.
   */
  public boolean atPose(Pose2d pose) {
    return atPose(pose, Translation.TOLERANCE, Rotation.TOLERANCE);
  }

  /**
   * Sets the states of each swerve module using target speeds that the drivetrain will work to
   * reach with applied acceleration limits to ensure smooth and safe operation.
   *
   * @param desired The robot relative speeds the drivetrain will run at.
   * @param mode The control loop used to achieve those speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds desired, ControlMode mode) {
    Vector<N2> currentVelocity =
        VecBuilder.fill(
            robotRelativeChassisSpeeds().vxMetersPerSecond,
            robotRelativeChassisSpeeds().vyMetersPerSecond);

    Vector<N2> deltaV =
        VecBuilder.fill(desired.vxMetersPerSecond, desired.vyMetersPerSecond)
            .minus(currentVelocity);

    Vector<N2> limitedVelocity = currentVelocity.plus(skidAccelerationLimit(deltaV));
    // currentVelocity.plus(currentVelocity.norm() > 1e-6 ?
    // skidAccelerationLimit(desiredAcceleration) : desiredAcceleration);

    log("/Robot/drive/forward accel limit", skidAccelerationLimit(deltaV).norm());

    ChassisSpeeds newSpeeds =
        new ChassisSpeeds(
            limitedVelocity.get(0), limitedVelocity.get(1), desired.omegaRadiansPerSecond);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(newSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED.in(MetersPerSecond));
    setModuleStates(
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(kinematics.toChassisSpeeds(states), PERIOD.in(Seconds))),
        mode);
  }

  /**
   * Applies forward acceleration limiting to the desired acceleration based on the current
   * velocity. Limits the acceleration in the direction of the current velocity to prevent excessive
   * acceleration.
   *
   * @param desiredAccel The desired field-relative acceleration vector.
   * @return The adjusted acceleration vector after applying forward acceleration limits.
   */
  private Vector<N2> forwardAccelerationLimit(Vector<N2> deltaV) {
    Vector<N2> currVel =
        VecBuilder.fill(
            fieldRelativeChassisSpeeds().vxMetersPerSecond,
            fieldRelativeChassisSpeeds().vyMetersPerSecond);
    double limit =
        maxAccel.get()
            * PERIOD.in(Seconds)
            * (1 - Math.min(1, currVel.norm() / MAX_SPEED.in(MetersPerSecond)));
    log("/Robot/drive/accel limit", limit);
    Vector<N2> proj = deltaV.projection(currVel);
    if (proj.norm() > limit && proj.dot(currVel) > 0) {
      Vector<N2> parallel = proj.unit().times(limit);
      Vector<N2> perpendicular = deltaV.minus(proj);
      return parallel.plus(perpendicular);
    }
    return deltaV;
  }

  /**
   * Applies skid acceleration limiting based on the maximum allowed skid acceleration. Ensures that
   * the acceleration does not exceed the skid limit to prevent skidding.
   *
   * @param desiredAccel The desired field-relative acceleration vector.
   * @return The adjusted acceleration vector after applying skid acceleration limits.
   */
  private Vector<N2> skidAccelerationLimit(Vector<N2> deltaV) {
    return deltaV.norm() == 0
        ? deltaV
        : deltaV.unit().times(Math.min(deltaV.norm(), maxSkidAccel.get() * PERIOD.in(Seconds)));
  }

  /**
   * Sets the states of each of the swerve modules.
   *
   * @param desiredStates The desired SwerveModule states.
   * @param mode The method to use when controlling the drive motor.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, ControlMode mode) {
    if (desiredStates.length != modules.size()) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).updateSetpoint(desiredStates[i], mode);
    }
  }

  /**
   * Command factory that automatically path-follows, in a straight line, to a position on the
   * field.
   *
   * @param target The pose to reach.
   * @return The command to run the control loop until the pose is reached.
   */
  public Command driveTo(Supplier<Pose2d> target) {
    return run(() -> {
          Pose2d targetPose = target.get();
          Pose2d pose = pose();
          Vector<N3> difference =
              VecBuilder.fill(
                  pose.getX() - targetPose.getX(),
                  pose.getY() - targetPose.getY(),
                  MathUtil.angleModulus(
                          pose.getRotation().getRadians() - targetPose.getRotation().getRadians())
                      * RADIUS.in(Meters));
          double out = translationController.calculate(difference.norm(), 0);
          Vector<N3> velocities = difference.unit().times(out);
          log("/Robot/drive/driveTo goal", targetPose, Pose2d.struct);
          setChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  velocities.get(0),
                  velocities.get(1),
                  velocities.get(2) / RADIUS.in(Meters),
                  pose().getRotation()),
              ControlMode.CLOSED_LOOP_VELOCITY);
        })
        .until(() -> atPose(target.get(), Translation.TOLERANCE, Rotation.TOLERANCE))
        .andThen(stop())
        .withName("drive to pose");
  }

  /**
   * Command factory that automatically path-follows, in a straight line, to a position on the
   * field.
   *
   * @param goal The pose to reach.
   * @return The command to run the control loop until the pose is reached.
   */
  public Command driveTo(Pose2d goal) {
    return driveTo(() -> goal);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules.get(i).drivePosition() / WHEEL_RADIUS.in(Meters);
    }
    return values;
  }

  /**
   * @return If the robot is skidding.
   */
  @Logged
  public boolean isSkidding() {
    DoubleSummaryStatistics diffs =
        Arrays.stream(moduleStates())
            .mapToDouble(
                s ->
                    FieldConstants.fromPolarCoords(s.speedMetersPerSecond, s.angle)
                        .minus(
                            VecBuilder.fill(
                                robotRelativeChassisSpeeds().vxMetersPerSecond,
                                robotRelativeChassisSpeeds().vyMetersPerSecond))
                        .norm())
            .summaryStatistics();
    return diffs.getMax() - diffs.getMin() > Skid.THRESHOLD.in(MetersPerSecond);
  }

  /**
   * @return If the robot is colliding.
   */
  @Logged
  public boolean isColliding() {
    return gyro.acceleration().norm() > MAX_ACCEL.in(MetersPerSecondPerSecond) * 2;
  }

  /**
   * Checks whether any module is stalling.
   *
   * @return If any module is stalling.
   */
  @Logged
  public boolean isStalling() {
    return Arrays.stream(modulesStalling)
        .map(bs -> bs.getAsBoolean())
        .reduce(false, (a, b) -> a || b);
  }

  /** Resets all drive encoders to read a position of 0. */
  public void resetEncoders() {
    modules.forEach(ModuleIO::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return resetGyro(Rotation2d.kZero);
  }

  /** Sets the gyro reading of the robot to a specified rotation. */
  public Command resetGyro(Rotation2d rotation) {
    return runOnce(() -> gyro.reset(rotation)).withName("gyro reset");
  }

  /** Returns the module states. */
  @Logged
  public SwerveModuleState[] moduleStates() {
    return modules.stream().map(ModuleIO::state).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module states. */
  @Logged
  public SwerveModuleState[] moduleSetpoints() {
    return modules.stream().map(ModuleIO::desiredState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module positions. */
  @Logged
  public SwerveModulePosition[] modulePositions() {
    return modules.stream().map(ModuleIO::position).toArray(SwerveModulePosition[]::new);
  }

  /** Returns the robot-relative chassis speeds. */
  public ChassisSpeeds robotRelativeChassisSpeeds() {
    return kinematics.toChassisSpeeds(moduleStates());
  }

  /** Returns the field-relative chassis speeds. */
  public ChassisSpeeds fieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeChassisSpeeds(), heading());
  }

  /**
   * Drives the robot to a Choreo {@link SwerveSample}.
   *
   * @param sample The SwerveSample to drive the robot to.
   * @param rotation A goal rotation to drive to.
   */
  public void goToSample(SwerveSample sample, Rotation2d rotation) {
    Vector<N2> displacement =
        pose().getTranslation().minus(sample.getPose().getTranslation()).toVector();

    Vector<N2> result =
        VecBuilder.fill(sample.vx, sample.vy)
            .plus(
                displacement.norm() > 1e-4
                    ? displacement
                        .unit()
                        .times(translationController.calculate(displacement.norm(), 0))
                    : displacement.times(0));

    if (Double.isNaN(result.norm())) {
      FaultLogger.report(
          "Alignment interference",
          "Assisted Drive and Pathfinding are interfering with each other, causing a NaN result speed.\nSpeed defaulted to zero.",
          FaultType.WARNING);
      result = VecBuilder.fill(0, 0);
    }

    setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            result.get(0),
            result.get(1),
            rotationController.calculate(heading().minus(rotation).getRadians(), 0),
            heading()),
        DRIVE_MODE);
  }

  /**
   * Updates pose estimate based on vision-provided {@link EstimatedRobotPose}s.
   *
   * @param poses The pose estimates based on vision data.
   */
  public void updateEstimates(PoseEstimate... poses) {
    Pose3d[] loggedEstimates = new Pose3d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      loggedEstimates[i] = poses[i].estimatedPose().estimatedPose;
      odometry.addVisionMeasurement(
          poses[i].estimatedPose().estimatedPose.toPose2d(),
          poses[i].estimatedPose().timestampSeconds,
          poses[i].standardDev());
      field2d
          .getObject("Cam " + i + " Est Pose")
          .setPose(poses[i].estimatedPose().estimatedPose.toPose2d());
    }
    log("/Robot/drive/estimated poses", loggedEstimates, Pose3d.struct);
  }

  @Override
  public void periodic() {
    // update our heading in reality / sim
    Tracer.startTrace("drive periodic");
    if (Robot.isReal()) {
      LOCK.lock();
      try {
        double[] timestamps = modules.get(2).timestamps();

        // get the positions of all modules at a given timestamp [[module0 odometry], [module1
        // odometry], ...]
        SwerveModulePosition[][] allPositions = {
          modules.get(0).odometryData(),
          modules.get(1).odometryData(),
          modules.get(2).odometryData(),
          modules.get(3).odometryData(),
        };
        double[][] allGyro = gyro.odometryData();

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        Rotation2d angle;
        for (int i = 0; i < timestamps.length; i++) {
          for (int m = 0; m < modules.size(); m++) {
            modulePositions[m] = allPositions[m][i];
          }

          angle = Rotation2d.fromRotations(allGyro[0][i]);
          odometry.updateWithTime(timestamps[i], angle, modulePositions);
          lastPositions = modulePositions;
          lastHeading = angle;
        }
      } catch (RuntimeException e) {
        e.printStackTrace();
      } finally {
        LOCK.unlock();
      }
    } else {
      odometry.update(simRotation, modulePositions());
      lastPositions = modulePositions();
    }

    // update our simulated field poses
    field2d.setRobotPose(pose());

    SmartDashboard.putData("Drive Field", field2d);

    for (int i = 0; i < modules2d.length; i++) {
      var module = modules.get(i);
      var transform = new Transform2d(MODULE_OFFSET[i], module.position().angle);
      modules2d[i].setPose(pose().transformBy(transform));
    }

    if (TUNING) {
      translationController.setPID(translationP.get(), translationI.get(), translationD.get());
      rotationController.setPID(rotationP.get(), rotationI.get(), rotationD.get());
    }

    log(
        "/Robot/drive/command",
        Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));

    Tracer.endTrace();
  }

  @SuppressWarnings("PMD.ConfusingTernary")
  @Override
  public void simulationPeriodic() {
    simRotation =
        simRotation.rotateBy(
            Rotation2d.fromRadians(
                !Double.isNaN(robotRelativeChassisSpeeds().omegaRadiansPerSecond)
                    ? robotRelativeChassisSpeeds().omegaRadiansPerSecond * PERIOD.in(Seconds)
                    : 0));
  }

  /** Stops the drivetrain. */
  public Command stop() {
    return runOnce(() -> setChassisSpeeds(new ChassisSpeeds(), DRIVE_MODE));
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement. */
  public Command lock() {
    var front = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    var back = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(
        () ->
            setModuleStates(
                new SwerveModuleState[] {front, back, back, front},
                ControlMode.OPEN_LOOP_VELOCITY));
  }

  /**
   * Factory for our drive systems check.
   *
   * <p>Checks for properly functioning movement and speed / heading measurements.
   *
   * @return The test to run.
   */
  public Test systemsCheck() {
    ChassisSpeeds speeds = new ChassisSpeeds(1, 1, 0);
    Command testCommand =
        run(() -> setChassisSpeeds(speeds, ControlMode.OPEN_LOOP_VELOCITY)).withTimeout(0.75);
    Function<ModuleIO, TruthAssertion> speedCheck =
        m ->
            tAssert(
                () -> m.state().speedMetersPerSecond * Math.signum(m.position().angle.getCos()) > 1,
                "Drive Syst Check " + m.name() + " Module Speed",
                () -> "expected: >= 1; actual: " + m.state().speedMetersPerSecond);
    Function<ModuleIO, EqualityAssertion> atAngle =
        m ->
            eAssert(
                "Drive Syst Check " + m.name() + " Module Angle (degrees)",
                () -> 45,
                () -> Units.radiansToDegrees(atan(m.position().angle.getTan())),
                1);
    Set<Assertion> assertions =
        modules.stream()
            .flatMap(m -> Stream.of(speedCheck.apply(m), atAngle.apply(m)))
            .collect(Collectors.toSet());
    return new Test(testCommand, assertions);
  }

  @Override
  public void close() throws Exception {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();
    gyro.close();
  }
}
