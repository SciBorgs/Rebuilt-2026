package org.sciborgs1155.robot.hopper;

import org.sciborgs1155.lib.Beambreak;
import static org.sciborgs1155.robot.Ports.Hopper.BEAMBREAK;
import org.sciborgs1155.robot.Robot;
import static org.sciborgs1155.robot.hopper.HopperConstants.A;
import static org.sciborgs1155.robot.hopper.HopperConstants.D;
import static org.sciborgs1155.robot.hopper.HopperConstants.I;
import static org.sciborgs1155.robot.hopper.HopperConstants.INTAKING_VELOCITY;
import static org.sciborgs1155.robot.hopper.HopperConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.hopper.HopperConstants.P;
import static org.sciborgs1155.robot.hopper.HopperConstants.S;
import static org.sciborgs1155.robot.hopper.HopperConstants.V;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Hopper extends SubsystemBase implements AutoCloseable {
  private final HopperIO hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;
  
  private final PIDController pid = new PIDController(P, I, D);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V, A);

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
  public Command runHopper(double velocity) {
    return run(() -> update(velocity));
  }

  /**
   * @return returns a run command to spin the motors at {@value HopperConstants#INTAKING_VELOCITY} rad/sec
   */
  public Command intake() {
    return runHopper(INTAKING_VELOCITY);
  }

  /**
   * @return returns a run command to spin the motors at negative {@value HopperConstants#INTAKING_VELOCITY} rad/sec
   */
  public Command outtake() {
    return runHopper(-INTAKING_VELOCITY);
  }

  /**
   * @return returns a run command to set the voltage of the motor to 0
   */
  public Command stop() {
    return run(() -> hardware.setVoltage(0));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
