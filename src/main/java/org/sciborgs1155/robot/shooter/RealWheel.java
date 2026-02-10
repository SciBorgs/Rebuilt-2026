package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealWheel implements WheelIO {
  private final TalonFX leader, follower;

  /** Sets the TalonFX motor configurations */
  public RealWheel() {
    leader = new TalonFX(LEADER);
    follower = new TalonFX(FOLLOWER);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);

    config.Feedback.SensorToMechanismRatio = SENSOR_MECHANISM_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    follower.setControl(new Follower(LEADER, MotorAlignmentValue.Aligned));

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    FaultLogger.register(leader);
    TalonUtils.addMotor(leader);
    FaultLogger.register(follower);
    TalonUtils.addMotor(follower);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public double velocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    leader.close();
  }
}
