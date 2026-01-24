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

public final class Hopper extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor hardware;
  private final Beambreak beambreak;
  public final Trigger blocked;

  /**
   * @return Creates a real hopper or no hopper based on Robot.isReal()
   */
  public static Hopper create() {
    return Robot.isReal() ? new Hopper(realMotor(), Beambreak.real(BEAMBREAK)) : none();
  }

  /**
   * @return Non-real hopper object
   */
  public static Hopper none() {
    return new Hopper(SimpleMotor.none(), Beambreak.none());
  }

  /**
   * @return A simple motor with hardware configurations
   */
  private static SimpleMotor realMotor() {
    TalonFX motor = new TalonFX(MOTOR);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return SimpleMotor.talon(motor, config);
  }

  /**
   * @param hardware represents the motor
   * @param beambreak represents the beambreak
   */
  private Hopper(SimpleMotor hardware, Beambreak beambreak) {
    this.hardware = hardware;
    this.beambreak = beambreak;

    this.blocked = new Trigger(() -> !beambreak.getState());

    setDefaultCommand(runHopper(INTAKING_POWER));
  }

  /**
   * @param power Power of hopper motors
   * @return Run command that sets hopper motor power to power
   */
  public Command runHopper(double power) {
    return run(() -> hardware.set(power));
  }

  /**
   * @return Run motors at motor power for intake
   */
  public Command intake() {
    return runHopper(INTAKING_POWER);
  }

  /**
   * @return Run motors at motor power for outtake
   */
  public Command outtake() {
    return runHopper(-INTAKING_POWER);
  }

  /**
   * @return Run motors with no power (stop)
   */
  public Command stop() {
    return runHopper(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
