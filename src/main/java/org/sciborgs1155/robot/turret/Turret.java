package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.eAssert;
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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Constants;
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
  private final ProfiledPIDController controller = new ProfiledPIDController(P, I, D, CONSTRAINTS);

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
            new SysIdRoutine.Config(
                RAMP_RATE,
                STEP_VOLTAGE,
                TIME_OUT,
                (state) -> SignalLogger.writeString("turret state", state.toString())),
            new SysIdRoutine.Mechanism(v -> hardware.setVoltage(v.in(Volts)), null, this));

    setDefaultCommand(run(() -> hardware.setVoltage(0)).withName("stop"));

    SmartDashboard.putData(
        "Turret quasistatic clockwise", sysIdTest(SysIdTestType.QUASISTATIC, Direction.kForward));
    SmartDashboard.putData(
        "Turret quasistatic counterclockwise",
        sysIdTest(SysIdTestType.QUASISTATIC, Direction.kReverse));
    SmartDashboard.putData(
        "Turret dynamic clockwise", sysIdTest(SysIdTestType.DYNAMIC, Direction.kForward));
    SmartDashboard.putData(
        "Turret dynamic counterclockwise", sysIdTest(SysIdTestType.DYNAMIC, Direction.kReverse));
  }

  public Command goLeft() {
    return run(() -> hardware.setVoltage(1));
  }

  public Command goRight() {
    return run(() -> hardware.setVoltage(-1));
  }

  /**
   * Returns the angular position of the turret.
   *
   * @return The angular position of the turret.
   */
  @Logged
  public double position() {
    return hardware.position();
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

  /** Enum used to specify the type of sysId test. */
  public enum SysIdTestType {
    QUASISTATIC,
    DYNAMIC
  }

  /**
   * Runs system identification test given the type and direction.
   *
   * @param type Type of sysId test. Either quasistatic or dynamic. (SysIdTestType Enum)
   * @param direction Direction of the motor. Forward is clockwise while reverse is
   *     counterclockwise.
   */
  @NotLogged
  public Command sysIdTest(SysIdTestType type, Direction direction) {
    Command test =
        switch (type) {
          case QUASISTATIC -> sysIdRoutine.quasistatic(direction);
          case DYNAMIC -> sysIdRoutine.dynamic(direction);
        };

    Angle stopAngle =
        direction == Direction.kForward ? MAX_ANGLE.minus(TOLERANCE) : MIN_ANGLE.plus(TOLERANCE);

    return test.until(
        () ->
            direction == Direction.kForward
                ? hardware.position() >= stopAngle.in(Radians)
                : hardware.position() <= stopAngle.in(Radians));
  }

  public Command manualTurret(InputStream input) {
    return goTo(input
            .deadband(.15, 1)
            .scale(MAX_VELOCITY.in(RadiansPerSecond))
            .scale(2)
            .scale(Constants.PERIOD.in(Seconds))
            .rateLimit(MAX_ACCELERATION.in(RadiansPerSecondPerSecond))
            .add(() -> controller.getGoal().position))
        .withName("manual elevator");
  }

  /**
   * Applies voltage to the motor based on setpoint.
   *
   * @param double The position setpoint in radians.
   */
  public void update(double positionSetpoint) {
    double pidVolts =
        controller.calculate(
            hardware.position(),
            MathUtil.clamp(positionSetpoint, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)));
    double ffdVolts = feedforward.calculate(controller.getSetpoint().velocity);

    hardware.setVoltage(pidVolts + ffdVolts);
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
  public Test goToTest(DoubleSupplier goal) {
    Command testCommand = goTo(goal).until(this::atGoal).withTimeout(5);
    Set<Assertion> assertions =
        Set.of(eAssert("Hood system check", goal, this::position, TOLERANCE.in(Radians)));
    return new Test(testCommand, assertions);
  }

  @Override
  public void periodic() {
    var command = getCurrentCommand();
    LoggingUtils.log("Robot/turret/current command", command != null ? command.getName() : "None");
    LoggingUtils.log("Robot/turret/encoder A position", hardware.encoderA());
    LoggingUtils.log("Robot/turret/encoder B position", hardware.encoderB());

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
    visualizer.update(
        hardware.position(), controller.getGoal().position, controller.getSetpoint().position);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
