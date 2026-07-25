package org.sciborgs1155.robot.indexer;

import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.Ports.Indexer.BEAMBREAK;
import static org.sciborgs1155.robot.indexer.IndexerConstants.A;
import static org.sciborgs1155.robot.indexer.IndexerConstants.D;
import static org.sciborgs1155.robot.indexer.IndexerConstants.I;
import static org.sciborgs1155.robot.indexer.IndexerConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.indexer.IndexerConstants.P;
import static org.sciborgs1155.robot.indexer.IndexerConstants.S;
import static org.sciborgs1155.robot.indexer.IndexerConstants.V;

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

public final class Indexer extends SubsystemBase implements AutoCloseable {
  private final IndexerIO hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;

  @Logged private final PIDController pid = new PIDController(P, I, D);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V, A);

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/indexer/P", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/indexer/I", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/indexer/D", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/indexer/S", S);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/indexer/V", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/indexer/A", A);

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
   * @return returns a run command to spin the motors at {@value IndexerConstants#RADIANS_PER_SEC}
   *     rad/sec
   */
  public Command forward() {
    return runIndexer(() -> IndexerConstants.RADIANS_PER_SEC);
    // return run(() -> hardware.setVoltage(6));
  }

  /**
   * @return returns a run command to spin the motors at negative {@value
   *     IndexerConstants#RADIANS_PER_SEC} rad/sec
   */
  public Command backward() {
    return runIndexer(() -> -IndexerConstants.RADIANS_PER_SEC);
    // return run(() -> hardware.setVoltage(-6));
  }

  /**
   * @return returns a run command to set the voltage of the motor to 0
   */
  public Command stop() {
    return run(() -> hardware.setVoltage(0));
  }

  /**
   * @return angular velocity of the indexer in rad/s
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
