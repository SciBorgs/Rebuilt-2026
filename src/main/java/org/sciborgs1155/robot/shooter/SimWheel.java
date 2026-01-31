package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.ControlConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimWheel implements WheelIO {
  private final FlywheelSim flywheel;

  /** Creates an instance of the flywheel motor. */
  public SimWheel() {
    flywheel =
        new FlywheelSim(LinearSystemId.identifyVelocitySystem(V, A), DCMotor.getKrakenX60(1));
  }

  /**
   * Sets the voltage of the motor.
   *
   * @return The desired voltage as a double.
   */
  @Override
  public void setVoltage(double voltage) {
    System.out.println(voltage);
    flywheel.setInputVoltage(voltage);
    flywheel.update(PERIOD.in(Seconds));
  }

  /**
   * Returns the velocity of the motor as a double.
   *
   * @return The velocity of the motor.
   */
  @Override
  public double velocity() {
    return flywheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
