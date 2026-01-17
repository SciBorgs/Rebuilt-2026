package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.hood.HoodConstants.DEFAULT_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.POS_TOLERANCE;
import static org.sciborgs1155.robot.hood.HoodConstants.RAMP_RATE;
import static org.sciborgs1155.robot.hood.HoodConstants.STEP_VOLTAGE;
import static org.sciborgs1155.robot.hood.HoodConstants.TIME_OUT;
import static org.sciborgs1155.robot.hood.HoodConstants.kA;
import static org.sciborgs1155.robot.hood.HoodConstants.kD;
import static org.sciborgs1155.robot.hood.HoodConstants.kG;
import static org.sciborgs1155.robot.hood.HoodConstants.kI;
import static org.sciborgs1155.robot.hood.HoodConstants.kP;
import static org.sciborgs1155.robot.hood.HoodConstants.kS;
import static org.sciborgs1155.robot.hood.HoodConstants.kV;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Robot;

/** Hood subsystem for adjusting vertical shooting angle of the fuel */
public class Hood extends SubsystemBase implements AutoCloseable {

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
    SmartDashboard.putData(
        "Robot/hood/quasistatic forward",
        sysIdRoutine
            .quasistatic(Direction.kForward)
            .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("hood quasistatic forward"));
    SmartDashboard.putData(
        "Robot/hood/quasistatic backward",
        sysIdRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("hood quasistatic backward"));
    SmartDashboard.putData(
        "Robot/hood/dynamic forward",
        sysIdRoutine
            .dynamic(Direction.kForward)
            .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("hood dynamic forward"));
    SmartDashboard.putData(
        "Robot/hood/dynamic backward",
        sysIdRoutine
            .dynamic(Direction.kReverse)
            .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("hood dynamic backward"));
  }

  /**
   * returns a new hood subsystem, which will have hardware if hood is real and sim if not
   *
   * @return a real or sim hood subsystem
   */
  public static Hood create() {
    return new Hood(Robot.isReal() ? new RealHood() : new SimHood());
  }

  /**
   * returns a hood with no interface
   *
   * @return
   */
  public static Hood none() {
    return new Hood(new NoHood());
  }

  /**
   * gets the current angle of the hood
   *
   * @return the angle in radians
   */
  @Logged
  public double angle() {
    return hardware.angle();
  }

  /**
   * sets the voltage of the motor
   *
   * @param v
   */
  public void setVoltage(double v) {
    hardware.setVoltage(v);
  }

  /**
   * returns the angle setpoint of the hood
   *
   * @return the position of the setpoint
   */
  @Logged
  public double angleSetpoint() {
    return fb.getSetpoint().position;
  }

  /**
   * gets the current velocity of the hood
   *
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
   *
   * @param goal
   * @return a goTo command set the hood to goal angle
   */
  private Command goTo(Angle goal) {
    return goTo(() -> goal.in(Radians));
  }

  /**
   * @return Whether or not the elevator is at its desired state.
   */
  @Logged
  public boolean atGoal() {
    return fb.atGoal();
  }

  public boolean atPosition(double angle) {
    return Math.abs(angle - angle()) < POS_TOLERANCE.in(Radians);
  }

  public Command goTo(DoubleSupplier goal) {
    return run(() -> update(goal.getAsDouble())).withName("Hood GoTo");
  }

  /**
   * method to set the voltage of the motor based of ff and fb calculations
   *
   * @param position Goal angle for hood to reach
   */
  private void update(double position) {
    double goal = MathUtil.clamp(position, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double feedback = fb.calculate(angle(), goal);
    double feedforward =
        ff.calculate(fb.getSetpoint().position - MIN_ANGLE.in(Radians), fb.getSetpoint().velocity);
    hardware.setVoltage(feedback + feedforward);
  }

  public Test goToTest(Angle goal) {

    Command testCommand = goTo(goal).until(this::atGoal).withTimeout(5);
    Set<Assertion> assertions =
        Set.of(
            eAssert(
                "hood system check (angle)",
                () -> goal.in(Radians),
                this::angle,
                POS_TOLERANCE.in(Radians)));

    return new Test(testCommand, assertions);
  }

  /** closes the hood */
  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
