package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Indexer.*;
import static org.sciborgs1155.robot.indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Indexer extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor hardware;

  private Indexer(SimpleMotor hardware) {
    this.hardware = hardware;

    setDefaultCommand(null);
  }

  public static Indexer create() {
    return (Robot.isReal()) ? new Indexer(realMotor()) : none();
  }

  public static Indexer none() {
    return new Indexer(SimpleMotor.none());
  }

  public static SimpleMotor realMotor() {
    TalonFX motor = new TalonFX(MOTOR);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(motor, config);
  }

  public Command runIntake(double power) {
    return run(() -> hardware.set(power));
  }

  public Command forward() {
    return runIntake(INTAKE_POWER);
  }

  public Command backward() {
    return runIntake(-INTAKE_POWER);
  }

  public Command stop() {
    return runIntake(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
