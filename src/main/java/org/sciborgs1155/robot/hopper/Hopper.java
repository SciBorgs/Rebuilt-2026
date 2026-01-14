package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Hopper.*;
import static org.sciborgs1155.robot.hopper.HopperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;

  public static Hopper create() {
    return Robot.isReal() ? new Hopper(realMotor(), Beambreak.real(BEAMBREAK)) : none();
  }

  public static Hopper none() {
    return new Hopper(SimpleMotor.none(), Beambreak.none());
  }

  private static SimpleMotor realMotor() {
    TalonFX motor = new TalonFX(MOTOR);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = HopperConstants.CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(motor, config);
  }

  public Hopper(SimpleMotor hardware, Beambreak beambreak) {
    this.hardware = hardware;
    this.beambreak = beambreak;

    this.blocked = new Trigger(() -> !beambreak.get());

    setDefaultCommand(stop());
  }

  public Command runHopper(double power) {
    return run(() -> hardware.set(power));
  }

  public Command intake() {
    return runHopper(INTAKING_POWER);
  }

  public Command outtake() {
    return runHopper(-INTAKING_POWER);
  }

  public Command stop() {
    return runHopper(0);
  }

  @Override
  public void close() throws Exception {
      hardware.close();
  }
}
