package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.Test;

public class SimpleMotorTest {
  @Test
  public void test() throws Exception {
    setupTests();
    DCMotorSim motor =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(1, 0.3), DCMotor.getKrakenX60(1));

    assert motor.getAngularAcceleration().in(RadiansPerSecondPerSecond) == 0;

    SimpleMotor sm = new SimpleMotor(motor::setInput, () -> {});
    run(Commands.run(() -> motor.update(0.02)));

    sm.set(0.5);
    assertEquals(motor.getInput().get(0, 0), 0.5);

    fastForward();
    assertEquals(motor.getAngularVelocityRadPerSec(), 0.5, 2e-3);

    sm.set(-0.5);

    fastForward();
    assertEquals(motor.getAngularVelocityRadPerSec(), -0.5, 2e-3);

    reset();
  }
}
