package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Indexer.*;
import static org.sciborgs1155.robot.indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public final class Indexer extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;

  /**
   * @return Creates a real indexer or no indexer based on Robot.isReal()
   */
  public static Indexer create() {
    return Robot.isReal() ? new Indexer(realMotor(), Beambreak.real(BEAMBREAK)) : none();
  }

  /**
   * @return non-real indexer
   */
  public static Indexer none() {
    return new Indexer(SimpleMotor.none(), Beambreak.none());
  }

  /**
   * @param hardware represents motor
   * @param beambreak represents beambreak
   */
  private Indexer(SimpleMotor hardware, Beambreak beambreak) {
    this.hardware = hardware;
    this.beambreak = beambreak;

    this.blocked = new Trigger(() -> !beambreak.getState());

    setDefaultCommand(stop());
  }

  /**
   * @return simple motor with hardware config
   */
  public static SimpleMotor realMotor() {
    TalonFX motor = new TalonFX(MOTOR);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(motor, config);
  }

  /**
   * @param power Power of indexer
   * @return run command that sets indexer motor power to power
   */
  public Command runIndexer(double power) {
    return run(() -> hardware.set(power));
  }

  /**
   * @return runs indexer with intake power
   */
  public Command forward() {
    return runIndexer(INTAKE_POWER);
  }

  /**
   * @return runs indexer in the opposite direction
   */
  public Command backward() {
    return runIndexer(-INTAKE_POWER);
  }

  /**
   * @return runs indexer withno power
   */
  public Command stop() {
    return runIndexer(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
