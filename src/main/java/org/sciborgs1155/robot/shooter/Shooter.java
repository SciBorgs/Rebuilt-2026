package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.ControlConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;

public final class Shooter extends SubsystemBase implements AutoCloseable {
  private final WheelIO hardware;

  @Logged private double setpoint;

  @Logged private final PIDController controller = new PIDController(P, I, D);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(S, V, A, PERIOD.in(Seconds));

  /**
   * Sets the shooter's default command and PID tolerance.
   *
   * @param hardware Takes in the WheelIO class.
   */
  public Shooter(WheelIO hardware) {
    this.hardware = hardware;

    controller.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    setDefaultCommand(runShooter(IDLE_VELOCITY.in(RadiansPerSecond)).withName("Idle"));
  }

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
    return new Shooter(new NoWheel());
  }

  /**
   * Returns the velocity of the motor.
   *
   * @return Return the value of the velocity.
   */
  @Logged
  public double getVelocity() {
    return hardware.velocity();
  }

  /**
   * Updates the velocity setpoint of the motor.
   *
   * @param velocitySetpoint The value of the velocity setpoint,
   */
  public void update(double velocitySetpoint) {
    double velocity =
        MathUtil.clamp(
            velocitySetpoint,
            -MAX_VELOCITY.in(RadiansPerSecond),
            MAX_VELOCITY.in(RadiansPerSecond));
    double ffVolts = feedforward.calculate(velocity); // feedforward
    double pidVolts = controller.calculate(getVelocity(), velocity);
    hardware.setVoltage(MathUtil.clamp(pidVolts + ffVolts, -MAX_VOLTAGE, MAX_VOLTAGE));
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
    return Math.abs(velocity - getVelocity()) < VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  public double setpoint() {
    return setpoint;
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

  /** Closes the shooter motor. */
  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
