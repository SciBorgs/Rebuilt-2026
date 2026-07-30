package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.ControlConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public final class Shooter extends SubsystemBase implements AutoCloseable {
  private final WheelIO hardware;

  @Logged
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          P, I, D, new TrapezoidProfile.Constraints(MAX_ACCELERATION, MAX_JERK));

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(S, V, A, PERIOD.in(Seconds));
  private final SysIdRoutine characterization;

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/shooter/K_P", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/shooter/K_I", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/shooter/K_D", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/shooter/S", S);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/shooter/V", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/shooter/A", A);

  /**
   * Returns the shooter subsystem
   *
   * @return Creates real or simulated shooter based on {@link Robot#isReal()}.
   */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
  }

  /**
   * Returns the shooter subsystem
   *
   * @return A shooter that is blank.
   */
  public static Shooter none() {
    return new Shooter(new Wheel());
  }

  /**
   * Sets the shooter's default command and PID tolerance.
   *
   * @param hardware Takes in the WheelIO class.
   */
  public Shooter(WheelIO hardware) {
    this.hardware = hardware;

    controller.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    characterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(10.0),
                Seconds.of(11),
                (state) -> SignalLogger.writeString("shooter state", state.toString())),
            new SysIdRoutine.Mechanism(
                v -> hardware.setVoltage(v.in(Volts)), null, this, "shooter"));

    SmartDashboard.putData(
        "Robot/shooter/shooter quasistatic backward",
        characterization.quasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "Robot/shooter/shooter quasistatic forward",
        characterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Robot/shooter/shooter dynamic backward", characterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "Robot/shooter/shooter dynamic forward", characterization.dynamic(Direction.kForward));

    setDefaultCommand(idleShooter());
  }

  /**
   * Updates the velocity setpoint of the motor.
   *
   * @param velocitySetpoint The value of the velocity setpoint (rad/s).
   * @param noDeceleration If true, sends no negative voltage, using power to break the motor. it'll
   *     at most just let it coast.
   */
  public void update(double velocitySetpoint, boolean noDeceleration) {
    double velocity =
        MathUtil.clamp(
            velocitySetpoint,
            -MAX_VELOCITY.in(RadiansPerSecond),
            MAX_VELOCITY.in(RadiansPerSecond));
    double lastVelocity = controller.getSetpoint().position;
    double pidVolts = controller.calculate(velocity(), velocity);
    double ffVolts =
        feedforward.calculateWithVelocities(lastVelocity, controller.getSetpoint().position);
    double volts = MathUtil.clamp(pidVolts + ffVolts, -MAX_VOLTAGE, MAX_VOLTAGE);
    hardware.setVoltage(noDeceleration ? Math.max(0, volts) : volts);
  }

  /**
   * Updates the velocity setpoint of the motor.
   *
   * @param velocitySetpoint The value of the velocity setpoint (rad/s).
   */
  public void update(double velocitySetpoint) {
    update(velocitySetpoint, false);
  }

  /**
   * Checks if the PID position is at the velocity setpoint
   *
   * @return PID position is at setpoint
   */
  @Logged
  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  /**
   * Checks if the current velocity and the target velocity is less than the velocity tolerance
   *
   * @return true or false
   */
  public boolean atVelocity(double velocity) {
    return Math.abs(velocity - velocity()) < VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  /**
   * @return Returns the setpoint of the controller.
   */
  @Logged
  public double setpoint() {
    return controller.getSetpoint().position;
  }

  /**
   * @return Returns the velocity of the shooter (Radians Per Second).
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity as a DoubleSupplier.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble())).withName("running shooter");
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity as a double.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  /**
   * Idles the shooter. It will not intentionally decelerate the wheels, but will keep the speed
   * above IDLE_VELOCITY.
   */
  public Command idleShooter() {
    return run(() -> update(0, true)).withName("Idle");
  }

  /**
   * Manual control of the shooter with an controller, which will be used for operator control and
   * testing.
   *
   * @param input The controller value to use for manual control.
   */
  public Command manualShooter(InputStream input) {
    return runShooter(
            input
                .deadband(() -> .15, 1)
                .scale(MAX_VELOCITY.in(RadiansPerSecond))
                .scale(PERIOD.in(Seconds))
                .add(() -> controller.getSetpoint().position))
        .withName("manual shooter");
  }

  /**
   * Does a test command to check if the subsystem works.
   *
   * @param goal The velocity goal.
   */
  public Command systemsCheck() {
    double goal = 200;

    return runShooter(goal)
        .until(() -> atSetpoint())
        .withTimeout(5)
        .andThen(
            FaultLogger.reportEquals(
                "Shooter Syst Check Speed",
                () -> goal,
                this::velocity,
                VELOCITY_TOLERANCE.in(RadiansPerSecond)));
  }

  /** Closes the shooter motor. */
  @Override
  public void close() throws Exception {
    hardware.close();
  }

  @Override
  public void periodic() {
    var command = getCurrentCommand();
    LoggingUtils.log("Robot/shooter/current command", command != null ? command.getName() : "None");
    if (TUNING) {
      controller.setP(tuningP.get());
      controller.setI(tuningI.get());
      controller.setD(tuningD.get());
      feedforward.setKs(tuningS.get());
      feedforward.setKv(tuningV.get());
      feedforward.setKa(tuningA.get());
    }
  }
}
