package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Intake.*;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Intake extends SubsystemBase implements AutoCloseable {
  // hardware initialization
  private final SimpleMotor hardware;

  /**
   * @param hardware the hardware is the object that will be operated on
   */
  public Intake(SimpleMotor hardware) {
    this.hardware = hardware;
    setDefaultCommand(stop());
  }

  /**
   * @return a real intake if the intake is real and a simmed intake if it is not in order to
   *     simulate the arm
   */
  public static Intake create() {
    return Robot.isReal() ? new Intake(realMotor()) : new Intake(SimpleMotor.none());
  }

  /**
   * @return a simple motor that will run the rollers
   */
  public static SimpleMotor realMotor() {
<<<<<<< HEAD
    final TalonFX motor = new TalonFX(ROLLERS);
=======
    final TalonFX motor = new TalonFX(ROLLER);
>>>>>>> c68abe5f121ab17a8bb724b18f98bda8b8e23066
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return SimpleMotor.talon(motor, config);
  }

  /**
   * @return create a new intake without hardware
   */
  public static Intake none() {
    return new Intake(SimpleMotor.none());
  }

  /**
   * @return start the rollers in order to intake fuel
   */
<<<<<<< HEAD
  public Command spin(double power) {
=======
  public Command runRollers(double power) {
>>>>>>> c68abe5f121ab17a8bb724b18f98bda8b8e23066
    return run(() -> hardware.set(power));
  }

  public Command intake() {
<<<<<<< HEAD
    return run(() -> spin(INTAKE_POWER));
=======
    return run(() -> runRollers(INTAKE_POWER));
>>>>>>> c68abe5f121ab17a8bb724b18f98bda8b8e23066
  }

  /**
   * @return stop the motors
   */
  public Command stop() {
<<<<<<< HEAD
    return run(() -> spin(0));
=======
    return run(() -> runRollers(0));
>>>>>>> c68abe5f121ab17a8bb724b18f98bda8b8e23066
  }

  /** close the hardware */
  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
