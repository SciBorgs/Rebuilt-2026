package org.sciborgs1155.robot.shooter;

import java.util.function.DoubleSupplier;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.robot.Ports.Shooter.*;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final WheelIO motor;

  /** Creates real or simulated shooter based on {@link Robot#isReal()}. */
  public static Shooter create() {
    return Robot.isReal()
        ? new Shooter(new RealWheel())
        : new Shooter(new SimWheel(kV, kA));
  }

  /** Creates a fake shooter. */
  public static Shooter none() {
    return new Shooter(new NoWheel());
  }

  public Shooter(WheelIO motor) {
    this.motor = motor;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public double getVelocity() {
    return motor.velocity();
  }

  public void update(double velocitySetpoint) {
    // pid
  }

  public boolean atSetpoint() {
    return true;
  }

  public boolean atVelocity(double velocity) {
    return true;
  }

  public double setpoint() {
    return 0.0;
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocity The desired velocity in radians per second.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return null;
  }

  public Command manualShooter(DoubleSupplier stickInput) {
    return null;
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