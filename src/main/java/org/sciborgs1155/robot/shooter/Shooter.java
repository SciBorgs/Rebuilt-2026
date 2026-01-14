package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final WheelIO motor;

  @Logged private double setpoint;

  @Logged private final PIDController pid = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(kS, kV, kA, PERIOD.in(Seconds));

  public Shooter(WheelIO motor) {
    this.motor = motor;

    pid.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    setDefaultCommand(
        runOnce(
                () -> {
                  motor.setVoltage(0);
                })
            .withName("Idle"));
  }

  /** Creates real or simulated shooter based on {@link Robot#isReal()}. */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel(kV, kA));
  }

  /** Creates a fake shooter. */
  public static Shooter none() {
    return new Shooter(new NoWheel());
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Logged
  public double getVelocity() {
    return motor.velocity();
  }

  public void update(double velocitySetpoint) {
    double FF = ff.calculate(velocitySetpoint);
    double FB = pid.calculate(getVelocity(), velocitySetpoint);
    motor.setVoltage(FB + FF);
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

  public Command ejectStuck(double velocity) {
    return runShooter(velocity);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
