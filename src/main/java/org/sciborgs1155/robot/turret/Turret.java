package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

public class Turret extends SubsystemBase {
  /**
   * Factory for the {@code Turret} subsystem. The hardware interface is varied depending on the
   * type of robot being operated on.
   *
   * @return A newly instantiated instance of {@code Turret} using a {@code RealTurret} hardware
   *     interface if the robot is real and a {@code SimTurret} hardware interface is the robot is
   *     simulated.
   */
  public static Turret create() {
    return Robot.isReal() ? new Turret(new RealTurret()) : new Turret(new SimTurret());
  }

  /**
   * Factory for a fake {@code Turret} subsystem.
   *
   * @return A newly instantiated instance of {@code Turret} using a {@code NoTurret} hardware
   *     interface.
   */
  public static Turret none() {
    return new Turret(new NoTurret());
  }

  @SuppressWarnings("unused")
  private final TurretIO motor;

  @SuppressWarnings("unused")
  private final ProfiledPIDController controller;

  @SuppressWarnings("unused")
  private final SimpleMotorFeedforward feedforward;

  public Turret(TurretIO turretIO) {
    motor = turretIO;
    feedforward = new SimpleMotorFeedforward(FFD.S, FFD.V, FFD.A, PERIOD.in(Seconds));
    controller =
        new ProfiledPIDController(
            PID.P,
            PID.I,
            PID.D,
            new Constraints(
                MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCELERATION.in(RadiansPerSecondPerSecond)),
            PERIOD.in(Seconds));
  }
}
