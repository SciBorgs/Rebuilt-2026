package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final WheelIO hardware;

  @Logged private double setpoint;

  @Logged private final PIDController pid = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(kS, kV, kA, PERIOD.in(Seconds));

  public Shooter(WheelIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    setDefaultCommand(runShooter(IDLE_VELOCITY.in(RadiansPerSecond)).withName("Idle"));
  }

  /** Creates real or simulated shooter based on {@link Robot#isReal()}. */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
  }

  /** Creates a fake shooter. */
  public static Shooter none() {
    return new Shooter(new NoWheel());
  }

  @Logged
  public double getVelocity() {
    return hardware.velocity();
  }

  public void update(double velocitySetpoint) {
    double velocity =
        Double.isNaN(velocitySetpoint)
            ? DEFAULT_VELOCITY.in(RadiansPerSecond)
            : MathUtil.clamp(
                velocitySetpoint,
                -MAX_VELOCITY.in(RadiansPerSecond),
                MAX_VELOCITY.in(RadiansPerSecond));
    double FF = ff.calculate(velocity); // feedforward
    double PID = pid.calculate(getVelocity(), velocity);
    hardware.setVoltage(MathUtil.clamp(PID + FF, -12, 12));
  }

  @Logged
  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public boolean atVelocity(double velocity) {
    return Math.abs(velocity - getVelocity()) < VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  public double setpoint() {
    return setpoint;
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity in radians per second.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble())).withName("running shooter");
  }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
