package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.Robot;

/**
 * The {@code Turret} subsystem consists of a single motor that is used to aim a variable hood
 * shooter at a specific target.
 */
public class Turret extends SubsystemBase implements AutoCloseable {
  /**
   * Factory for the {@code Turret} subsystem. The hardware interface is varied depending on the
   * type of robot being operated on.
   *
   * @return A newly instantiated instance of {@code Turret} using a {@code RealTurret} hardware
   *     interface if the robot is real and a {@code SimTurret} hardware interface is the robot is
   *     simulated.
   */
  @NotLogged
  public static Turret create() {
    return Robot.isReal() ? new Turret(new RealTurret()) : new Turret(new SimTurret());
  }

  /**
   * Factory for a fake {@code Turret} subsystem.
   *
   * @return A newly instantiated instance of {@code Turret} using a {@code NoTurret} hardware
   *     interface.
   */
  @NotLogged
  public static Turret none() {
    return new Turret(new NoTurret());
  }

  /** Motor used to rotate the turret. */
  @NotLogged private final TurretIO motor;

  /** {@code PIDController} used to orient the turret to a specified angle. */
  @NotLogged private final ProfiledPIDController controller;

  /** {@code Feedforward} used to aid in orienting the turret to a specified angle. */
  @NotLogged private final SimpleMotorFeedforward feedforward;

  public Turret(TurretIO turretIO) {
    motor = turretIO;
    feedforward = new SimpleMotorFeedforward(FF.S, FF.V, FF.A, PERIOD.in(Seconds));

    controller = new ProfiledPIDController(PID.P, PID.I, PID.D, PID.CONSTRAINTS);
    controller.setTolerance(
        PID.POSITION_TOLERANCE.in(Radians), PID.VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }

  /**
   * Updates turret orientation setpoint. This setpoint will be used to determine the voltage that
   * is fed to the motors periodically.
   *
   * @param angle The angle to orient the turret towards (front of robot is 0 DEG).
   */
  public void orient(Angle angle) {} // TODO: Implement.

  /** Updates the motor voltage based on the setpoint specified by the {@code orient} method. */
  @Override
  public void periodic() {
    // TODO: VOLTAGE SETTING

    // LOGGING
    LoggingUtils.log("POSITION", motor.position());
    LoggingUtils.log("VELOCITY", motor.velocity());
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
