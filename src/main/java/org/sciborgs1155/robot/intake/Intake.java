package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Radians;

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

public class Intake extends SubsystemBase implements AutoCloseable {
  // hardware initialization
  private final IntakeIO hardware;
  public static TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          IntakeConstants.MAX_VELOCITY, IntakeConstants.MAX_ACCELERATION);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, constraints);

  private final ArmFeedforward ff =
      new ArmFeedforward(
          IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);

  // TODO set up IntakeIO Constructor so this sequence works
  /** (1) method signature (input: IntakeIO) (output: Intake class) (2) this.hardware = hardware */

  /**
   * @param hardware the hardware is the object that will be operated on
   */
  public Intake(IntakeIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(IntakeConstants.POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.extensionPosition());
    pid.setGoal(IntakeConstants.START_ANGLE);
  }

  /**
   * @return a real intake if the intake is real and a simmed intake if it is not in order to
   *     simulate the arm
   */
  public static Intake create() {
    if (Robot.isReal()) {
      return new Intake(new RealIntake());
    } else {
      return new Intake(new SimIntake());
    }
  }

  /**
   * @return create a new intake without hardware
   */
  public static Intake none() {
    return new Intake(new NoIntake());
  }

  // TODO
  /** Command Factory: */
  /**
   * @return set the voltage in order to extend the intake
   */
  public Command setArmVoltage(double voltage) {
    return Commands.runOnce(() -> hardware.setArmVoltage(voltage), this);
  }

  /**
   * @param angle go to angle
   * @return command to make the intake go to the angle
   */
  public Command goTo(Angle angle) {
    return runOnce(
            () -> {
              double rads =
                  MathUtil.clamp(
                      angle.in(Radians),
                      IntakeConstants.MIN_ANGLE.in(Radians),
                      IntakeConstants.MAX_ANGLE.in(Radians));

              pid.setGoal(rads);
            })
        .andThen(
            run(
                () -> {
                  double output = pid.calculate(hardware.extensionPosition());
                  hardware.setArmVoltage(output);
                }));
  }

  /**
   * @return start the rollers in order to intake fuel
   */
  public Command startRollers() {
    return Commands.run(() -> hardware.setRollerVoltage());
  }

  /**
   * @param angle test if the intake will go to the angle
   * @return the test
   */
  public Test goToTest(Angle angle) {
    Command testCommand = goTo(angle).until(() -> pid.atGoal()).withTimeout(8);
    EqualityAssertion atGoal =
        Assertion.eAssert(
            "arm angle",
            () -> angle.in(Radians),
            () -> hardware.extensionPosition(),
            IntakeConstants.POSITION_TOLERANCE.in(Radians));
    return new Test(testCommand, Set.of(atGoal));
  }

  @Override
  public void close() throws Exception {}

  /**
   * @param angle set the intake to be at said angle
   */
  public void periodic(Angle angle) {
    hardware.setArmVoltage(
        pid.calculate(
                hardware.extensionPosition(),
                MathUtil.clamp(
                    angle.in(Radians),
                    IntakeConstants.MIN_ANGLE.in(Radians),
                    IntakeConstants.MAX_ANGLE.in(Radians)))
            + ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }
}
