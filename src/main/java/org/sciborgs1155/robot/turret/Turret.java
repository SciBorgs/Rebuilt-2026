package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.ControlConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.Robot;

/**
 * The {@code Turret} subsystem consists of a single motor that is used to aim a variable hood
 * shooter at a specific target.
 */
public class Turret extends SubsystemBase implements AutoCloseable {
  /**
   * Factory for the {@code Turret} subsystem. The hardware interface is varied depending on the
   * type of robot being operated on.
   *
   * @return A newly instantiated instance of {@code Turret} using a {@code RealTurret} hardware
   *     interface if the robot is real and a {@code SimTurret} hardware interface is the robot is
   *     simulated.
   */
  @Logged private double setpoint;

  // * Logs previous velocity. */
  @Logged private double lastVelocity;

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

  /** Motor used to rotate the turret. */
  @NotLogged private final TurretIO hardware;

  /** {@code PIDController} used to orient the turret to a specified angle. */
  @NotLogged
  private final ProfiledPIDController controller =
      new ProfiledPIDController(kP, kI, kD, CONSTRAINTS);

  /** {@code Feedforward} used to aid in orienting the turret to a specified angle. */
  @NotLogged
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(kS, kV, kA, PERIOD.in(Seconds));

  /** Visualization. Green = Position, Red = Setpoint. */
  private final TurretVisualizer visualizer = new TurretVisualizer(6, 7);

  /** System identification routine object. */
  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructs a new turret subsystem.
   *
   * @param pivot The hardware implementation to use.
   */
  public Turret(TurretIO turretIO) {
    hardware = turretIO;

    controller.setTolerance(TOLERANCE.in(Radians));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
            new SysIdRoutine.Mechanism(
                v -> hardware.setVoltage(v.in(Volts)),
                log -> {
                  log.motor("turret")
                      .angularPosition(Radians.of(hardware.position()))
                      .angularVelocity(RadiansPerSecond.of(hardware.velocity()));
                },
                this));

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

  /**
   * Returns the angular position of the turret.
   *
   * @return The angular position of the turret.
   */
  public double position() {
    return hardware.position();
  }

  /**
   * Returns the angular setpoint of the turret specified by the {@code setAngle} method.
   *
   * @return The angular setpoint of the turret.
   */
  public Angle setpoint() {
    return Radians.of(controller.getSetpoint().position);
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
  public Command sysIdTest(SysIdTestType type, Direction direction) {
    /*
    Angle startAngle = direction == Direction.kForward ? MIN_ANGLE : MAX_ANGLE;

    Command goToStart =
        Commands.runOnce(() -> setAngle(startAngle)).andThen(run().until(controller::atGoal));
    */

    Command test =
        switch (type) {
          case QUASISTATIC -> sysIdRoutine.quasistatic(direction);
          case DYNAMIC -> sysIdRoutine.dynamic(direction);
        };

    Angle stopAngle =
        direction == Direction.kForward ? MAX_ANGLE.minus(TOLERANCE) : MIN_ANGLE.plus(TOLERANCE);

    double sign = direction == Direction.kForward ? 1.0 : -1.0;

    /*
    return goToStart.andThen(
        test.until(() -> sign * motor.position().in(Radians) >= sign * stopAngle.in(Radians)));
    */

    return test.until(() -> sign * hardware.position() >= sign * stopAngle.in(Radians));
  }

  /**
   * Applies voltage to the motor based on setpoint.
   *
   * @param double The position setpoint in radians.
   */
  public void update(double positionSetpoint) {
    double currentPosition = hardware.position();
    double pidVolts = controller.calculate(currentPosition, positionSetpoint);

    double targetVelocity = controller.getSetpoint().velocity;
    double ffdVolts = feedforward.calculate(lastVelocity, targetVelocity);

    hardware.setVoltage(pidVolts + ffdVolts);

    lastVelocity = targetVelocity;
    setpoint = positionSetpoint;
  }

  /**
   * Sets controller setpoint with a supplier and repeatively calls update to orient the turret.
   *
   * @param DoubleSupplier The position supplier.
   */
  public Command runTurret(DoubleSupplier position) {
    return runOnce(() -> controller.setGoal(position.getAsDouble()))
        .andThen(run(() -> update(position.getAsDouble())));
  }

  /**
   * Sets controller setpoint and repeatively calls update to orient the turret.
   *
   * @param Double The position.
   */
  public Command runTurret(Double position) {
    return runTurret(() -> position);
  }

  @Override
  public void periodic() {
    // LOGGING
    LoggingUtils.log("Robot/Turret/POSITION", hardware.position());
    LoggingUtils.log("Robot/Turret/VELOCITY", hardware.velocity());
    LoggingUtils.log("Robot/Turret/VOLTAGE", hardware.voltage());
    LoggingUtils.log("Robot/Turret/SETPOINT", controller.getSetpoint().position);

    // VISUALIZATION
    visualizer.update(hardware.position(), controller.getSetpoint().position);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
