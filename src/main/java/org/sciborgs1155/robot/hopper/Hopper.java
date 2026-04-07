package org.sciborgs1155.robot.hopper;

import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.Ports.Hopper.BEAMBREAK;
import static org.sciborgs1155.robot.hopper.HopperConstants.A;
import static org.sciborgs1155.robot.hopper.HopperConstants.D;
import static org.sciborgs1155.robot.hopper.HopperConstants.I;
import static org.sciborgs1155.robot.hopper.HopperConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.hopper.HopperConstants.P;
import static org.sciborgs1155.robot.hopper.HopperConstants.S;
import static org.sciborgs1155.robot.hopper.HopperConstants.V;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public final class Hopper extends SubsystemBase implements AutoCloseable {
  private final HopperIO hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;

  @Logged private final PIDController pid = new PIDController(P, I, D);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V, A);

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/hopper/P", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/hopper/I", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/hopper/D", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/hopper/S", S);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/hopper/V", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/hopper/A", A);

  /**
   * @return Creates a real hopper or no hopper based on Robot.isReal()
   */
  public static Hopper create() {
    return Robot.isReal() ? new Hopper(new RealHopper(), Beambreak.real(BEAMBREAK)) : none();
  }

  /**
   * @return Non-real hopper object
   */
  public static Hopper none() {
    return new Hopper(new NoHopper(), Beambreak.none());
  }

  /**
   * @param hardware represents the motor
   * @param beambreak represents the beambreak
   */
  private Hopper(HopperIO hardware, Beambreak beambreak) {
    this.hardware = hardware;
    this.beambreak = beambreak;

    this.blocked = new Trigger(() -> !beambreak.getState());

    setDefaultCommand(stop());
  }

  /**
   * @param setpoint The velocity setpoint to run the motor at in rad/sec
   */
  public void update(double setpoint) {
    double feedforward = ff.calculate(setpoint);
    double feedback = pid.calculate(hardware.velocity(), setpoint);
    double voltage = feedforward + feedback;
    hardware.setVoltage(MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE));
  }

  /**
   * @param velocity velocity of hopper motors in radians/sec
   * @return Run command that sets given velocity to motor velocity
   */
  public Command runHopper(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble()));
  }

  /**
   * @return returns a run command to spin the motors at {@value HopperConstants#RADIANS_PER_SEC}
   *     rad/sec
   */
  public Command intake() {
    return runHopper(() -> HopperConstants.RADIANS_PER_SEC);
  }

  /**
   * @return returns a run command to spin the motors at negative {@value
   *     HopperConstants#RADIANS_PER_SEC} rad/sec
   */
  public Command outtake() {
    return runHopper(() -> -HopperConstants.RADIANS_PER_SEC);
  }

  /**
   * @return returns a run command to set the voltage of the motor to 0
   */
  public Command stop() {
    return run(() -> hardware.setVoltage(0));
  }

  /**
   * @return angular velocity of the hopper in rad/s
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  @Override
  public void periodic() {
    if (TUNING) {
      pid.setP(tuningP.get());
      pid.setI(tuningI.get());
      pid.setD(tuningD.get());
      ff.setKs(tuningS.get());
      ff.setKv(tuningV.get());
      ff.setKa(tuningA.get());
    }
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
