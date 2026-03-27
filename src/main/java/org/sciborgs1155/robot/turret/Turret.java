package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.tAssert;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.turret.TurretConstants.CONSTRAINTS;
import static org.sciborgs1155.robot.turret.TurretConstants.CRT_MATCH_TOLERANCE;
import static org.sciborgs1155.robot.turret.TurretConstants.ENCODER_A_GEARING;
import static org.sciborgs1155.robot.turret.TurretConstants.ENCODER_A_OFFSET;
import static org.sciborgs1155.robot.turret.TurretConstants.ENCODER_B_GEARING;
import static org.sciborgs1155.robot.turret.TurretConstants.ENCODER_B_OFFSET;
import static org.sciborgs1155.robot.turret.TurretConstants.MAX_ACCELERATION;
import static org.sciborgs1155.robot.turret.TurretConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.turret.TurretConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.turret.TurretConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.turret.TurretConstants.RAMP_RATE;
import static org.sciborgs1155.robot.turret.TurretConstants.STEP_VOLTAGE;
import static org.sciborgs1155.robot.turret.TurretConstants.TIME_OUT;
import static org.sciborgs1155.robot.turret.TurretConstants.TURRET_GEARING;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.turret.TurretConstants.PositionControlConstants;
import org.sciborgs1155.robot.turret.TurretConstants.VelocityControlConstants;

/**
 * The {@code Turret} subsystem consists of a single motor that is used to aim a variable hood
 * shooter at a specific target.
 */
@Logged(name = "turret")
public final class Turret extends SubsystemBase implements AutoCloseable {
  /** Motor used to rotate the turret. */
  @NotLogged public final TurretIO hardware;

  /** {@code PIDController} used to orient the turret to a specified angle. */
  @Logged
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          PositionControlConstants.P,
          PositionControlConstants.I,
          PositionControlConstants.D,
          CONSTRAINTS);

  /** {@code PIDController} used to drive the turret at a specified velocity. */
  @Logged
  private final PIDController velocityController =
      new PIDController(
          VelocityControlConstants.P, VelocityControlConstants.I, VelocityControlConstants.D);

  /** {@code Feedforward} used to aid in orienting the turret to a specified angle. */
  @Logged
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          PositionControlConstants.S,
          PositionControlConstants.V,
          PositionControlConstants.A,
          PERIOD.in(Seconds));

  /** {@code Feedforward} used to aid in driving the turret at a specified velocity. */
  @Logged
  private final SimpleMotorFeedforward velocityFeedforward =
      new SimpleMotorFeedforward(
          VelocityControlConstants.S,
          VelocityControlConstants.V,
          VelocityControlConstants.A,
          PERIOD.in(Seconds));

  /** Visualization. Green = Position, Red = Setpoint. */
  private final TurretVisualizer visualizer = new TurretVisualizer(6, 7);

  /** System identification routine object. */
  private final SysIdRoutine sysIdRoutine;

  @NotLogged
  private final DoubleEntry tuningP =
      Tuning.entry("Robot/tuning/turret/K_P", PositionControlConstants.P);

  @NotLogged
  private final DoubleEntry tuningI =
      Tuning.entry("Robot/tuning/turret/K_I", PositionControlConstants.I);

  @NotLogged
  private final DoubleEntry tuningD =
      Tuning.entry("Robot/tuning/turret/K_D", PositionControlConstants.D);

  @NotLogged
  private final DoubleEntry tuningS =
      Tuning.entry("Robot/tuning/turret/S", PositionControlConstants.S);

  @NotLogged
  private final DoubleEntry tuningV =
      Tuning.entry("Robot/tuning/turret/V", PositionControlConstants.V);

  @NotLogged
  private final DoubleEntry tuningA =
      Tuning.entry("Robot/tuning/turret/A", PositionControlConstants.A);

  @NotLogged
  private final DoubleEntry tuningVelocityP =
      Tuning.entry("Robot/tuning/turret/velocity/K_P", VelocityControlConstants.P);

  @NotLogged
  private final DoubleEntry tuningVelocityI =
      Tuning.entry("Robot/tuning/turret/velocity/K_I", VelocityControlConstants.I);

  @NotLogged
  private final DoubleEntry tuningVelocityD =
      Tuning.entry("Robot/tuning/turret/velocity/K_D", VelocityControlConstants.D);

  @NotLogged
  private final DoubleEntry tuningVelocityS =
      Tuning.entry("Robot/tuning/turret/velocity/S", VelocityControlConstants.S);

  @NotLogged
  private final DoubleEntry tuningVelocityV =
      Tuning.entry("Robot/tuning/turret/velocity/V", VelocityControlConstants.V);

  @NotLogged
  private final DoubleEntry tuningVelocityA =
      Tuning.entry("Robot/tuning/turret/velocity/A", VelocityControlConstants.A);

  private double lastGoodPositionRad;

  /** CRT solver — called once at startup to seed the motor encoder. */
  private final CustomCRT crt;

  /** Delays CRT until CANcoders have had time to boot and report valid readings. */
  private final Timer startupTimer = new Timer();

  /** True once CRT has successfully seeded the motor encoder. */
  private boolean seeded;

  /** Creates real or simulated turret based on {@link Robot#isReal()}. */
  @NotLogged
  public static Turret create() {
    return Robot.isReal() ? new Turret(new RealTurret()) : new Turret(new SimTurret());
  }

  /**
   * Factory for a fake {@code Turret} subsystem.
   *
   * @return A newly instantiated instance of {@code Turret} using a {@code NoTurret} hardware
   *     interface.
   */
  @NotLogged
  public static Turret none() {
    return new Turret(new NoTurret());
  }

  /**
   * Constructs a new turret subsystem.
   *
   * @param pivot The hardware implementation to use.
   */
  private Turret(TurretIO turretIO) {
    hardware = turretIO;

    controller.setTolerance(PositionControlConstants.TOLERANCE.in(Radians));

    crt =
        new CustomCRT(
            TURRET_GEARING / ENCODER_A_GEARING,
            TURRET_GEARING / ENCODER_B_GEARING,
            MIN_ANGLE.in(Rotations),
            MAX_ANGLE.in(Rotations),
            CRT_MATCH_TOLERANCE.in(Rotations),
            true,
            true,
            ENCODER_A_OFFSET.in(Rotations),
            ENCODER_B_OFFSET.in(Rotations));

    startupTimer.start();

    sysIdRoutine =
        new SysIdRoutine(
            new Config(
                RAMP_RATE,
                STEP_VOLTAGE,
                TIME_OUT,
                (state) -> SignalLogger.writeString("turret state", state.toString())),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
    SmartDashboard.putData(
        "Robot/turret/quasistatic clockwise",
        sysIdRoutine.quasistatic(Direction.kForward).withName("turret quasistatic clockwise"));
    SmartDashboard.putData(
        "Robot/turret/quasistatic counterclockwise",
        sysIdRoutine
            .quasistatic(Direction.kReverse)
            .withName("turret quasistatic counterclockwise"));
    SmartDashboard.putData(
        "Robot/turret/dynamic clockwise",
        sysIdRoutine.dynamic(Direction.kForward).withName("turret dynamic clockwise"));
    SmartDashboard.putData(
        "Robot/turret/dynamic counterclockwise",
        sysIdRoutine.dynamic(Direction.kReverse).withName("turret dynamic counterclockwise"));

    setDefaultCommand(run(() -> hardware.setVoltage(0)).withName("stop"));
  }

  /** manual control to test the turret, makes it go left. */
  public Command goLeft() {
    return run(() -> hardware.setVoltage(1));
  }

  /** manual control to test the turret, makes it go right. */
  public Command goRight() {
    return run(() -> hardware.setVoltage(-1));
  }

  /**
   * Returns the angular position of the turret in radians.
   *
   * <p>Returns 0 until the startup delay has elapsed and CRT first solves successfully. After that,
   * reflects the most recent CRT result from {@code periodic()}, falling back to the last good
   * value on solver failure.
   *
   * @return The angular position of the turret.
   */
  @Logged
  public double position() {
    return lastGoodPositionRad;
  }

  /**
   * Returns the setpoint of the turret.
   *
   * @return The setpoint of the turret.
   */
  @Logged
  public double setpoint() {
    return controller.getSetpoint().position;
  }

  /**
   * Returns whether the turret is at its goal or not.
   *
   * @return whether the turret is at its goal or not.
   */
  @Logged
  public boolean atGoal() {
    return controller.atGoal();
  }

  /**
   * manual control of the turret with an controller, which will be used for operator control and
   * testing
   *
   * @param input The controller value to use for manual control.
   */
  public Command manualTurret(InputStream input) {
    return goTo(input
            .deadband(.15, 1)
            .scale(MAX_VELOCITY.in(RadiansPerSecond))
            .scale(2)
            .scale(PERIOD.in(Seconds))
            .rateLimit(MAX_ACCELERATION.in(RadiansPerSecondPerSecond))
            .add(() -> controller.getSetpoint().position))
        .withName("manual turret");
  }

  /**
   * Applies voltage to the motor based on setpoint.
   *
   * @param double The position setpoint in radians.
   */
  private void update(double positionSetpoint) {
    double pos = position();
    double pidVolts =
        controller.calculate(
            pos, MathUtil.clamp(positionSetpoint, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)));
    double ffdVolts = feedforward.calculate(controller.getSetpoint().velocity);

    double voltage = pidVolts + ffdVolts;

    if (pos >= MAX_ANGLE.in(Radians)) voltage = Math.min(voltage, 0);
    if (pos <= MIN_ANGLE.in(Radians)) voltage = Math.max(voltage, 0);

    hardware.setVoltage(voltage);
  }

  /**
   * Applies voltage to the motor based on setpoint.
   *
   * @param double The velocity setpoint in radians.
   */
  private void updateVelocity(double velocitySetpoint) {
    double pos = position();
    double pidVolts = velocityController.calculate(hardware.velocity(), velocitySetpoint);
    double ffVolts = velocityFeedforward.calculate(velocitySetpoint);
    double voltage = pidVolts + ffVolts;

    if (pos >= MAX_ANGLE.in(Radians)) voltage = Math.min(voltage, 0);
    if (pos <= MIN_ANGLE.in(Radians)) voltage = Math.max(voltage, 0);

    hardware.setVoltage(voltage);
  }

  /**
   * Moves the turret to the closest valid position matching the given yaw supplier, handling the
   * 90° overlap region by choosing whichever equivalent position requires less travel.
   *
   * @param yaw The target yaw as a Rotation2d supplier.
   */
  public Command goToYaw(Supplier<Rotation2d> yaw) {
    return goTo(
        () -> {
          double theta = yaw.get().getRadians();
          double c1 = theta;
          double c2 = theta - 2 * Math.PI;
          boolean c1Valid = c1 >= MIN_ANGLE.in(Radians) && c1 <= MAX_ANGLE.in(Radians);
          boolean c2Valid = c2 >= MIN_ANGLE.in(Radians) && c2 <= MAX_ANGLE.in(Radians);
          if (c1Valid && c2Valid) {
            double pos = position();
            return Math.abs(c1 - pos) <= Math.abs(c2 - pos) ? c1 : c2;
          }
          return c1Valid ? c1 : c2;
        });
  }

  /**
   * Moves the turret to the closest valid position matching the given yaw supplier, handling the
   * 90° overlap region by choosing whichever equivalent position requires less travel.
   *
   * @param yaw The target yaw as a Rotation2d.
   */
  public Command goToYaw(Rotation2d yaw) {
    return goToYaw(() -> yaw);
  }

  /**
   * Sets controller setpoint with a supplier and repeatedly calls update to orient the turret.
   *
   * @param DoubleSupplier The position supplier.
   */
  public Command goTo(DoubleSupplier position) {
    return run(() -> update(position.getAsDouble())).withName("goTo (DoubleSupplier)");
  }

  /**
   * Drives the turret at the specified angular velocity in radians per second.
   *
   * @param DoubleSupplier The velocity supplier.
   */
  public Command goToVelocity(DoubleSupplier goalVelocity) {
    return run(() -> updateVelocity(goalVelocity.getAsDouble())).withName("goToVelocity");
  }

  /**
   * Returns the angular velocity of the turret in radians per second.
   *
   * @return the current velocity in radians per second.
   */
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * Test for turret to go to a set goal angle.
   *
   * @param goal The goal in radians.
   */
  public Test goToTest(DoubleSupplier goal) {
    Command testCommand = goTo(goal).until(this::atGoal).withTimeout(5);
    Set<Assertion> assertions =
        Set.of(
            tAssert(
                this::atGoal,
                "Turret system check",
                () ->
                    String.format(
                        "Turret goal check: current=%.3f rad, goal=%.3f rad, tolerance=%.3f rad",
                        position(),
                        goal.getAsDouble(),
                        PositionControlConstants.TOLERANCE.in(Radians))));
    return new Test(testCommand, assertions);
  }

  /**
   * Attempts CRT seed after the startup delay has elapsed. Runs once; on success seeds the motor
   * encoder and sets {@code seeded = true}. Reports a fault if CRT fails to find a solution.
   */
  private void trySeedFromCRT() {
    crt.solve(hardware.encoderA(), hardware.encoderB())
        .ifPresentOrElse(
            rot -> {
              lastGoodPositionRad = rot * 2.0 * Math.PI;
              seeded = true;
              SmartDashboard.putNumber("Robot/turret/crt seed rot", rot);
            },
            () ->
                FaultLogger.report(
                    new Fault(
                        "Turret CRT seed failed",
                        String.format(
                            "CRT could not resolve position from encoderA=%.4f encoderB=%.4f",
                            hardware.encoderA(), hardware.encoderB()),
                        FaultType.WARNING)));
  }

  @Override
  public void periodic() {
    if (!seeded && startupTimer.hasElapsed(3.0)) {
      // Wait for CANcoders to boot before attempting CRT seed.
      trySeedFromCRT();
    } else if (seeded) {
      // Update position once per loop from CRT.
      crt.solve(hardware.encoderA(), hardware.encoderB())
          .ifPresent(rot -> lastGoodPositionRad = rot * 2.0 * Math.PI);
    }

    SmartDashboard.putBoolean("Robot/turret/crt seeded", seeded);

    var command = getCurrentCommand();
    LoggingUtils.log("Robot/turret/current command", command != null ? command.getName() : "None");
    LoggingUtils.log("Robot/turret/encoder A position", hardware.encoderA());
    LoggingUtils.log("Robot/turret/encoder B position", hardware.encoderB());
    LoggingUtils.log("Robot/turret/crt'd position", position());
    LoggingUtils.log("Robot/turret/velocity", velocity());

    if (hardware instanceof SimTurret sim) {
      LoggingUtils.log("Robot/turret/true angle", sim.trueAngleRad());
    }

    hardware.periodic();

    if (TUNING) {
      controller.setP(tuningP.get());
      controller.setI(tuningI.get());
      controller.setD(tuningD.get());
      feedforward.setKs(tuningS.get());
      feedforward.setKv(tuningV.get());
      feedforward.setKa(tuningA.get());
      velocityController.setP(tuningVelocityP.get());
      velocityController.setI(tuningVelocityI.get());
      velocityController.setD(tuningVelocityD.get());
      velocityFeedforward.setKs(tuningVelocityS.get());
      velocityFeedforward.setKv(tuningVelocityV.get());
      velocityFeedforward.setKa(tuningVelocityA.get());
    }

    // VISUALIZATION
    visualizer.update(position(), controller.getGoal().position, controller.getSetpoint().position);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
