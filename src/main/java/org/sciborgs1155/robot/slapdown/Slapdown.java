package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Set;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

@Logged
public class Slapdown extends SubsystemBase implements AutoCloseable {
  // hardware initialization
  private final SlapdownIO hardware;

  private final ProfiledPIDController pid = new ProfiledPIDController(P, I, D, CONSTRAINTS);

  private final ArmFeedforward ff = new ArmFeedforward(S, G, V, A);

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/slapdown/tuningP", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/slapdown/tuningI", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/slapdown/tuningD", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/slapdown/tuningS", S);
  @NotLogged private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/slapdown/tuningG", G);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/slapdown/tuningV", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/slapdown/tuningA", A);

  /** Routine for recording and analyzing motor data. */
  private final SysIdRoutine sysIdRoutine;

  /**
   * @param hardware the hardware is the object that will be operated on
   */
  public Slapdown(SlapdownIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE.in(Radians));

    setDefaultCommand(retract());

    sysIdRoutine =
        new SysIdRoutine(
            new Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
    SmartDashboard.putData(
        "Robot/slapdown/quasistatic forward",
        sysIdRoutine
            .quasistatic(Direction.kForward)
            .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("slapdown quasistatic forward"));
    SmartDashboard.putData(
        "Robot/slapdown/quasistatic backward",
        sysIdRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("slapdown quasistatic backward"));
    SmartDashboard.putData(
        "Robot/slapdown/dynamic forward",
        sysIdRoutine
            .dynamic(Direction.kForward)
            .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("slapdown dynamic forward"));
    SmartDashboard.putData(
        "Robot/slapdown/dynamic backward",
        sysIdRoutine
            .dynamic(Direction.kReverse)
            .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("slapdown dynamic backward"));
  }

  /**
   * @return a real Slapdown if the Slapdown is real and a simmed Slapdown if it is not in order to
   *     simulate the arm
   */
  public static Slapdown create() {
    return new Slapdown(Robot.isReal() ? new RealSlapdown() : new SimSlapdown());
  }

  /**
   * @return a non-implemented slapdown
   */
  public static Slapdown none() {
    return new Slapdown(new NoSlapdown());
  }

  /**
   * Command Factory
   *
   * @param angle go to angle
   * @return command to make the Slapdown go to the angle
   */
  public Command goTo(double angle) {
    return run(() -> update(angle)).withName("go to angle");
  }

  /**
   * @return slap down the intake
   */
  public Command extend() {
    return goTo(MIN_ANGLE.in(Radians));
  }

  /**
   * @return bring up the intake
   */
  public Command retract() {
    return goTo(MAX_ANGLE.in(Radians));
  }

  /**
   * @return the position of the slapdown
   */
  @Logged
  public double position() {
    return hardware.position();
  }

  /**
   * @return the position of the pid
   */
  @Logged
  public double setpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * @param angle test if the Slapdown is at said angle
   * @return the test
   */
  public boolean atPosition(double angle) {
    return Math.abs(angle - position()) < POSITION_TOLERANCE.in(Radians);
  }

  /**
   * checks whether the slapdown is at a set desired state
   *
   * @return Whether or not the slapdown is at its desired state.
   */
  @Logged
  public boolean atGoal() {
    return pid.atGoal();
  }

  /**
   * @param angle set the Slapdown to be at said angle
   */
  public void update(double angle) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double pidVoltage = pid.calculate(hardware.position(), rads);
    double ffVoltage = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidVoltage + ffVoltage);
  }

  /**
   * @param angle test if the Slapdown will go to the angle
   * @return the test
   */
  public Test goToTest(double angle) {
    EqualityAssertion atGoal =
        Assertion.eAssert(
            "Slapdown angle", () -> angle, hardware::position, POSITION_TOLERANCE.in(Radians));
    Command testCommand = goTo(angle).until(pid::atGoal).withTimeout(5);
    return new Test(testCommand, Set.of(atGoal));
  }

  @Override
  public void periodic() {
    if (TUNING) {
      pid.setP(tuningP.get());
      pid.setI(tuningI.get());
      pid.setD(tuningD.get());
      ff.setKg(tuningG.get());
      ff.setKa(tuningA.get());
      ff.setKv(tuningV.get());
      ff.setKs(tuningS.get());
    }
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
