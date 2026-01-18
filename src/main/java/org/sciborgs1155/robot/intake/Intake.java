package org.sciborgs1155.robot.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.sciborgs1155.robot.Robot;

public class Intake extends SubsystemBase implements AutoCloseable {
  // hardware initialization
  private final IntakeIO hardware;
  public static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(IntakeConstants.MAX_VELOCITY, IntakeConstants.MAX_ACCELERATION);
  private final ProfiledPIDController pid = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, constraints);

  private final ArmFeedforward ff = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);

  // TODO set up IntakeIO Constructor so this sequence works
  /** (1) method signature (input: IntakeIO) (output: Intake class) (2) this.hardware = hardware */
  
  /**
   * 
   * @param hardware the hardware is the object that will be operated on
   */
  public Intake(IntakeIO hardware) {
    this.hardware = hardware;
  }

  /**
   * 
   * @return a real intake if the intake is real and a simmed intake if it is not in order to simulate the arm
   */
  public static Intake create() {
    if (Robot.isReal()) {
      return new Intake(new RealIntake());
    } else {
      return new Intake(new SimIntake());
    }
  }

  /**
   * 
   * @return create a new intake without hardware
   */
  public static Intake none(){
    return new Intake(new NoIntake());
  }

  // TODO
  /** Command Factory: */
  /**
   * 
   * @return set the voltage in order to extend the intake
   */
  public Command setArmVoltage() {
    return Commands.runOnce(() -> hardware.setArmVoltage(), this);
  }

  /**
   * 
   * @return start the rollers in order to intake fuel
   */
  public Command startRollers() {
    return Commands.run(() -> hardware.setRollerVoltage());
  }

  @Override
  public void close() throws Exception {}

}
