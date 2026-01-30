package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

/** Hood subsystem for adjusting vertical shooting angle of the fuel */
@Logged
public class Hood extends SubsystemBase implements AutoCloseable {

  private final HoodIO hardware;

  @Logged
  private final ProfiledPIDController fb =
      new ProfiledPIDController(
          K_P,
          K_I,
          K_D,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));

  /** Arm feed forward controller. */
  private final ArmFeedforward ff = new ArmFeedforward(K_S, K_G, K_V, K_A);

  private final HoodVisualizer measurement =
      new HoodVisualizer("measurement", new Color8Bit(Color.kBlue), new Color8Bit(Color.kDarkBlue));
  private final HoodVisualizer setpoint =
      new HoodVisualizer("setpoint", new Color8Bit(Color.kOrange), new Color8Bit(Color.kOrangeRed));

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/hood/K_P", K_P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/hood/K_I", K_I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/hood/K_D", K_D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/hood/K_S", K_S);
  @NotLogged private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/hood/K_G", K_G);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/hood/K_V", K_V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/hood/K_A", K_A);

  /** Routine for recording and analyzing motor data. */
  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructor
   *
   * @param hardware : The HoodIO object (real/simulated/nonexistent) that will be operated on.
   */
  public Hood(HoodIO hardware) {
    this.hardware = hardware;

    fb.setTolerance(POS_TOLERANCE.in(Radians));
    fb.reset(angle());
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

  /**
   * returns the velocity setpoint of the hood
   *
   * @return the velocity of the setpoint
   */
  @Logged
  public double velocitySetpoint() {
    return fb.getSetpoint().velocity;
  }

  /**
   * checks whether the hood is at a set desired state
   *
   * @return Whether or not the elevator is at its desired state.
   */
  @Logged
  public boolean atGoal() {
    return fb.atGoal();
  }

  /** checks if the hood is at a certain position within tolerance */
  public boolean atPosition(double angle) {
    return Math.abs(angle - angle()) < POS_TOLERANCE.in(Radians);
  }

  /**
   * moves the hood to a specified angle
   *
   * @param goal
   * @return a goTo command set the hood to goal angle
   */
  public Command goTo(Angle goal) {
    return goTo(() -> goal.in(Radians));
  }

  /** makes hood go to a set goal position */
  public Command goTo(DoubleSupplier goal) {
    return run(() -> update(goal.getAsDouble())).withName("Hood GoTo");
  }

  /**
   * goes to an angle so that the fuel is launch out at said angle
   *
   * @param angle angle to shoot at
   * @return a command to go to the shooting angle
   */
  public Command goToShootingAngle(DoubleSupplier goal) {
    return run(() -> update(goal.getAsDouble() - SHOOTING_ANGLE_OFFSET.in(Radians)))
        .until(this::atGoal)
        .withName("Hood GoTo Angle");
  }

  /**
   * goes to an angle so that the fuel is launch out at said angle
   *
   * @param angle angle to shoot at
   * @return a command to go to the shooting angle
   */
  public Command goToShootingAngle(Angle goal) {
    return goToShootingAngle(() -> goal.in(Radians));
  }

  /**
   * method to set the voltage of the motor based of ff and fb calculations
   *
   * @param position Goal angle for hood to reach
   */
  private void update(double position) {
    double goal = MathUtil.clamp(position, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double feedback = fb.calculate(angle(), goal);
    double feedforward = ff.calculate(fb.getSetpoint().position, fb.getSetpoint().velocity);
    hardware.setVoltage(feedback + feedforward);
  }

  /** test for hood to go to a set goal angle */
  public Test goToTest(Angle goal) {
    Command testCommand = goTo(goal).until(this::atGoal).withTimeout(5);
    Set<Assertion> assertions =
        Set.of(
            eAssert(
                "Hood system check",
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

  @Override
  public void periodic() {
    measurement.setAngle(Radians.of(angle()).in(Degrees));
    setpoint.setAngle(Radians.of(angleSetpoint()).in(Degrees));
    if (TUNING.get()) {
      fb.setP(tuningP.get());
      fb.setI(tuningI.get());
      fb.setD(tuningD.get());
      ff.setKs(tuningS.get());
      ff.setKg(tuningG.get());
      ff.setKv(tuningV.get());
      ff.setKa(tuningA.get());
    }
    LoggingUtils.log(
        "/Robot/hood/command",
        Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }
}
