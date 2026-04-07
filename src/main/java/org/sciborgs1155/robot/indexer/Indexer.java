package org.sciborgs1155.robot.indexer;

import org.sciborgs1155.lib.Beambreak;
import static org.sciborgs1155.robot.Ports.Indexer.BEAMBREAK;
import org.sciborgs1155.robot.Robot;
import static org.sciborgs1155.robot.indexer.IndexerConstants.A;
import static org.sciborgs1155.robot.indexer.IndexerConstants.D;
import static org.sciborgs1155.robot.indexer.IndexerConstants.I;
import static org.sciborgs1155.robot.indexer.IndexerConstants.INTAKING_VELOCITY;
import static org.sciborgs1155.robot.indexer.IndexerConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.indexer.IndexerConstants.P;
import static org.sciborgs1155.robot.indexer.IndexerConstants.S;
import static org.sciborgs1155.robot.indexer.IndexerConstants.V;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Indexer extends SubsystemBase implements AutoCloseable {
  private final IndexerIO hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;
  
  private final PIDController pid = new PIDController(P, I, D);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V, A);

  /**
   * @return Creates a real indexer or no indexer based on Robot.isReal()
   */
  public static Indexer create() {
    return Robot.isReal() ? new Indexer(new RealIndexer(), Beambreak.real(BEAMBREAK)) : none();
  }

  /**
   * @return Non-real indexer object
   */
  public static Indexer none() {
    return new Indexer(new NoIndexer(), Beambreak.none());
  }

  /**
   * @param hardware represents the motor
   * @param beambreak represents the beambreak
   */
  private Indexer(IndexerIO hardware, Beambreak beambreak) {
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
   * @param velocity velocity of indexer motors in radians/sec
   * @return Run command that sets given velocity to motor velocity
   */
  public Command runIndexer(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble()));
  }

  /**
   * @return returns a run command to spin the motors at {@value IndexerConstants#INTAKING_VELOCITY} rad/sec
   */
  public Command intake() {
    return runIndexer(() -> INTAKING_VELOCITY);
  }

  /**
   * @return returns a run command to spin the motors at negative {@value IndexerConstants#INTAKING_VELOCITY} rad/sec
   */
  public Command outtake() {
    return runIndexer(() -> -INTAKING_VELOCITY);
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
