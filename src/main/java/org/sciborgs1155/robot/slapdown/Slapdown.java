package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;
import org.sciborgs1155.lib.Tuning;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Robot;
import static org.sciborgs1155.robot.Constants.*;

@Logged
public class Slapdown extends SubsystemBase implements AutoCloseable {
  // hardware initialization
  private final SlapdownIO hardware;

  private final ProfiledPIDController pid = new ProfiledPIDController(P, I, D, CONSTRAINTS);

  private final ArmFeedforward ff = new ArmFeedforward(S, G, V, A);

  private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/tuningP", P);
  private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/tuningI", I);
  private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/tuningD", D);
  
  private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/tuningS", S);
  private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/tuningG", G);
  private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/tuningV", V);
  private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/tuningA", A);

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
    return goTo(MAX_ANGLE.in(Radians));
  }

  /**
   * @return bring up the intake
   */
  public Command retract() {
    return goTo(MIN_ANGLE.in(Radians));
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
    Command testCommand = goTo(angle).until(pid::atGoal);
    return new Test(testCommand, Set.of(atGoal));
  }

  @Override
  public void periodic(){
    if (TUNING.get()){
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
