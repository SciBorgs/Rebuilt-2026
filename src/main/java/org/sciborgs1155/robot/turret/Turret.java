package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.turret.TurretConstants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.ControlConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

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
          P,
          I,
          D,
          new Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));

  /** {@code Feedforward} used to aid in orienting the turret to a specified angle. */
  @Logged
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(S, V, A, PERIOD.in(Seconds));

  /** Visualization. Green = Position, Red = Setpoint. */
  private final TurretVisualizer visualizer = new TurretVisualizer(6, 7);

  /** System identification routine object. */
  private final SysIdRoutine sysIdRoutine;

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/turret/K_P", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/turret/K_I", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/turret/K_D", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/turret/S", S);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/turret/V", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/turret/A", A);

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

    controller.setTolerance(TOLERANCE.in(Radians));

    sysIdRoutine =
        new SysIdRoutine(
            new Config(
                RAMP_RATE,
                STEP_VOLTAGE,
                TIME_OUT,
                state -> SignalLogger.writeString("turret state", state.toString())),
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
   * manual control to test the turret, makes it go in the direction of the joystick on the
   * controller
   *
   * @param x the x axis of the joystick
   * @param y the y axis of the joystick
   * @return a command to manually face the turret
   */
  public Command fromJoysticks(InputStream x, InputStream y) {
    return goToYaw(
        () ->
            Rotation2d.fromRadians(
                Math.hypot(x.getAsDouble(), y.getAsDouble()) > 0.1
                    ? Math.atan2(y.getAsDouble(), x.getAsDouble())
                    : position()));
  }

  /**
   * Returns the angular position of the turret in radians.
   *
   * @return The angular position of the turret.
   */
  @Logged
  public double position() {
    return hardware.angle();
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
   * Returns the goal of the turret.
   *
   * @return The goal of the turret.
   */
  @Logged
  public double goal() {
    return controller.getGoal().position;
  }

  /**
   * Returns whether the turret is at its goal or not.
   *
   * @return whether the turret is at its goal or not.
   */
  @Logged
  public boolean atGoal() {
    return Math.abs(controller.getGoal().position - position()) < TOLERANCE.in(Radians);
  }

  /** Enum used to specify the type of sysId test. */
  public enum SysIdTestType {
    QUASISTATIC,
    DYNAMIC
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
  public void update(double positionSetpoint) {
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
          double c2 = theta + 2 * Math.PI;
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
   * Moves the turret to the closest valid position matching the given yaw supplier, handling the
   * 90° overlap region by choosing whichever equivalent position requires less travel.
   *
   * @param yaw The target yaw as a Rotation2d.
   * @param heading The heading of the robot.
   */
  public Command goToFieldRelativeYaw(Supplier<Rotation2d> yaw, Supplier<Rotation2d> heading) {
    return goToYaw(() -> yaw.get().minus(heading.get())).withName("goToYaw field relative");
  }

  /**
   * Sets controller setpoint with a supplier and repeatively calls update to orient the turret.
   *
   * @param DoubleSupplier The position supplier.
   */
  public Command goTo(DoubleSupplier position) {
    return run(() -> update(position.getAsDouble())).withName("goTo (DoubleSupplier)");
  }

  /**
   * Test for turret to go to a set goal angle.
   *
   * @param goal The goal in radians.
   */
  public Command systemsCheck() {
    DoubleSupplier goal = () -> MAX_ANGLE.div(4).in(Radians);

    return goTo(goal)
        .until(this::atGoal)
        .withTimeout(5)
        .andThen(
            FaultLogger.reportEquals(
                "Hood system check", goal, this::position, TOLERANCE.in(Radians)))
        .andThen(goTo(() -> START_ANGLE.in(Radians)));
  }

  @Override
  public void periodic() {
    var command = getCurrentCommand();
    LoggingUtils.log("Robot/turret/current command", command != null ? command.getName() : "None");

    hardware.periodic();

    if (TUNING) {
      controller.setP(tuningP.get());
      controller.setI(tuningI.get());
      controller.setD(tuningD.get());
      feedforward.setKs(tuningS.get());
      feedforward.setKv(tuningV.get());
      feedforward.setKa(tuningA.get());
    }

    // VISUALIZATION
    visualizer.update(position(), controller.getGoal().position, controller.getSetpoint().position);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
