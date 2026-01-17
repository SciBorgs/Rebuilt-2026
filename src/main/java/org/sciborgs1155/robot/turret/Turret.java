package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.ControlConstants.*;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  @NotLogged private final TurretIO motor;

  /** {@code PIDController} used to orient the turret to a specified angle. */
  @NotLogged
  private final ProfiledPIDController controller =
      new ProfiledPIDController(PROPORTIONAL_GAIN, INTEGRAL_GAIN, DERIVATIVE_GAIN, CONSTRAINTS);

  /** {@code Feedforward} used to aid in orienting the turret to a specified angle. */
  @NotLogged
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN, PERIOD.in(Seconds));

  /** Visualization. Green = Position, Red = Setpoint */
  private final TurretVisualizer visualizer = new TurretVisualizer(6, 7);

  // TODO ADD
  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructs a new turret subsystem.
   *
   * @param pivot The hardware implementation to use.
   */
  public Turret(TurretIO turretIO) {
    motor = turretIO;

    controller.setTolerance(TOLERANCE.in(Radians));

    sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
          new SysIdRoutine.Mechanism(
              v -> motor.setVoltage(v), 
              log -> {
                log.motor("turret")
                    .angularPosition(motor.position())
                    .angularVelocity(motor.velocity());
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

    // setDefaultCommand(run());
  }

  /**
   * Updates turret orientation setpoint. This setpoint will be used to determine the voltage that
   * is fed to the motors periodically.
   *
   * @param angle The angle to orient the turret towards (front of robot is 0 DEG).
   */
  public void setAngle(Angle angle) {
    controller.setGoal(angle.in(Radians));
  }

  /**
   * Returns the angular position of the turret.
   *
   * @return The angular position of the turret.
   */
  public Angle position() {
    return motor.position();
  }

  /**
   * Returns the angular setpoint of the turret specified by the {@code setAngle} method.
   *
   * @return The angular setpoint of the turret.
   */
  public Angle setpoint() {
    return motor.position();
  }

  // TODO ADD SYSID STUFF
  private static final Angle MARGIN = Degrees.of(2);

  public enum SysIdTestType {
    QUASISTATIC,
    DYNAMIC
  }

  public Command sysIdTest(SysIdTestType type, Direction direction) {
    Angle startAngle = (direction == Direction.kForward) ? MIN_ANGLE : MAX_ANGLE;
    Angle endAngle = (direction == Direction.kForward) ? MAX_ANGLE : MIN_ANGLE;

    Command goToStart =
        Commands.runOnce(() -> setAngle(startAngle)).andThen(run().until(controller::atGoal));

    Command test =
        switch (type) {
          case QUASISTATIC -> sysIdRoutine.quasistatic(direction);
          case DYNAMIC -> sysIdRoutine.dynamic(direction);
        };

    return goToStart.andThen(test.until(() -> motor.position().isNear(endAngle, MARGIN)));
  }

  public void randomAngle() {
    Angle setpoint = Radians.of(Math.random() * Math.PI * 2).minus(Radians.of(Math.PI));
    setAngle(setpoint);
  }

  /**
   * Continuously orients the turret towards the angle setpoint specified in the {@code setAngle}
   * method.
   *
   * @return A command to continuously orient the turret towards the angle setpoint.
   */
  public Command run() {
    System.out.println("help");
    return run(
        () -> {
          // PID CONTROL (RADIANS)
          double targetPosition = motor.position().in(Radians);
          double pidVolts = controller.calculate(targetPosition);

          // FEEDFORWARD CONTROL (RADIANS/SEC)
          double targetVelocity = controller.getSetpoint().velocity;
          double ffdVolts = feedforward.calculate(targetVelocity);

          // VOLTAGE SETTING
          motor.setVoltage(Volts.of(pidVolts + ffdVolts));
        });
  }

  @Override
  public void periodic() {
    // LOGGING
    LoggingUtils.log("Robot/Turret/POSITION", motor.position());
    LoggingUtils.log("Robot/Turret/VELOCITY", motor.velocity());
    LoggingUtils.log("Robot/Turret/SETPOINT", controller.getSetpoint().position);

    // VISUALIZATION
    visualizer.update(motor.position().in(Radians), controller.getSetpoint().position);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
