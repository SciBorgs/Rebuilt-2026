package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;

public class Hood extends SubsystemBase {

  private final HoodIO hardware;

  private final ProfiledPIDController fb =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));

  /** Arm feed forward controller. */
  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  /** Routine for recording and analyzing motor data. */
  private final SysIdRoutine sysIdRoutine;

  public Hood(HoodIO hardware) {
    this.hardware = hardware;

    fb.setTolerance(POS_TOLERANCE.in(Radians));
    setDefaultCommand(goTo(DEFAULT_ANGLE));

    sysIdRoutine =
        new SysIdRoutine(
            new Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
  }

  public static Hood create() {
    return new Hood(Robot.isReal() ? new RealHood() : new SimHood());
  }

  public static Hood none() {
    return new Hood(new NoHood());
  }

  /**
   * gets the current angle of the hood
   * @return the angle in radians
   */
  @Logged
  public double angle() {
    return hardware.angle().in(Radians);
  }

  /**
   * sets the voltage of the motor 
   * @param v
   */
  public void setVoltage(double v) {
    hardware.setVoltage(v);
  }

  /**
   * returns the angle setpoint of the hood
   * @return the position of the setpoint
   */
  @Logged
  public double angleSetpoint() {
    return fb.getSetpoint().position;
  }

  /**
   * gets the current velocity of the hood
   * @return current velocity of the hood
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  @Logged
  public double velocitySetpoint() {
    return fb.getSetpoint().velocity;
  }

  /**
   * moves the hood to a specified angle
   * @param goal
   * @return a goTo command set the hood to goal angle
   */
  private Command goTo(Angle goal) {
    return goTo(() -> goal.in(Radians));
  }


  private Command goTo(DoubleSupplier goal) {
    return run(
        () -> {
          double feedback = fb.calculate(angle(), goal.getAsDouble());
          double feedforward = ff.calculate(angleSetpoint(), velocitySetpoint());
          hardware.setVoltage(feedback + feedforward);
        });
  }

  /**
   * moves the hood to a specified exit angle for the projectile
   * @param angle
   * @return a goTo command setting t
   */
  public Command goToShootingAngle(DoubleSupplier angle) {
    return goTo(
        () ->
            ((angle.getAsDouble() - HOOD_ANGLE.in(Radians) - Math.PI / 2)
                * HOOD_RADIUS
                / MOTOR_RADIUS));
  }

  public Command goToShootingAngle(Angle angle) {
    return goToShootingAngle(() -> angle.in(Radians));
  }
}
