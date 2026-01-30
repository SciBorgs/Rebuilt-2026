package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Robot;

@Logged
public class Slapdown extends SubsystemBase implements AutoCloseable {
  // hardware initialization
  private final SlapdownIO hardware;

  public static TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
  private final ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, constraints);

  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  // TODO set up SlapdownIO Constructor so this sequence works
  /**
   * (1) method signature (input: SlapdownIO) (output: Slapdown class) (2) this.hardware = hardware
   */

  /**
   * @param hardware the hardware is the object that will be operated on
   */
  public Slapdown(SlapdownIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE.in(Radians));

    setDefaultCommand(retract());
  }

  /**
   * @return a real Slapdown if the Slapdown is real and a simmed Slapdown if it is not in order to
   *     simulate the arm
   */
  public static Slapdown create() {
    return Robot.isReal() ? new Slapdown(new RealSlapdown()) : new Slapdown(new SimSlapdown());
  }

  public static Slapdown none() {
    return new Slapdown(new NoSlapdown());
  }

  // TODO
  /** Command Factory: */
  /**
   * @return set the voltage in order to extend the Slapdown
   */
  public Command setArmVoltage(double voltage) {
    return Commands.runOnce(() -> hardware.setVoltage(voltage), this);
  }

  /**
   * @param angle go to angle
   * @return command to make the Slapdown go to the angle
   */
  public Command goTo(Angle angle) {
    return run(
        () -> {
          double rads =
              MathUtil.clamp(angle.in(Radians), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));

          double output = pid.calculate(rads, hardware.position());
          hardware.setVoltage(output);
        });
  }

  /**
   * @return slap down the intake
   */
  public Command extend() {
    return goTo(MAX_ANGLE);
  }

  /**
   * @return bring up the intake
   */
  public Command retract() {
    return goTo(MIN_ANGLE);
  }

  @Logged
  /**
   * @return the position of the slapdown
   */
  public double position() {
    return hardware.position();
  }

  @Logged
  /**
   * @return the position of the pid
   */
  public double setpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * @param angle test if the Slapdown will go to the angle
   * @return the test
   */
  public Test goToTest(Angle angle) {
    EqualityAssertion atGoal =
        Assertion.eAssert(
            "Slapdown angle",
            () -> angle.in(Radians),
            hardware::position,
            POSITION_TOLERANCE.in(Radians));
    Command testCommand = goTo(angle).until(pid::atGoal);
    return new Test(testCommand, Set.of(atGoal));
  }

  @Override
  public void close() throws Exception {}

  /**
   * @param angle set the Slapdown to be at said angle
   */
  public void periodic(Angle angle) {
    hardware.setVoltage(
        pid.calculate(
                hardware.position(),
                MathUtil.clamp(angle.in(Radians), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)))
            + ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }
}
