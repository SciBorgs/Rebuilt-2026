package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.tuning;
import static org.sciborgs1155.robot.climb.ClimbConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

@Logged
public final class Climb extends SubsystemBase implements AutoCloseable {
  private final ClimbIO hardware;
  private final SysIdRoutine sysIdRoutine;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          P,
          I,
          D,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  private final ElevatorFeedforward ff = new ElevatorFeedforward(S, G, V);

  private final ClimbVisualizer setpoint =
      new ClimbVisualizer("setpoint visualizer", new Color8Bit(0, 0, 255));

  private final ClimbVisualizer measurement =
      new ClimbVisualizer("measurement visualizer", new Color8Bit(255, 0, 0));

  private final DoubleEntry kS = Tuning.entry("/Robot/tuning/elevator/kS", S);
  private final DoubleEntry kG = Tuning.entry("/Robot/tuning/elevator/kG", G);
  private final DoubleEntry kV = Tuning.entry("/Robot/tuning/elevator/kV", V);
  private final DoubleEntry kA = Tuning.entry("/Robot/tuning/elevator/kA", A);

  /**
   * @return A climb object that either returns a climb with hardware or a simulated climb depending
   *     on if the robot exists or not.
   */
  public static Climb create() {
    return new Climb(Robot.isReal() ? new RealClimb() : new SimClimb());
  }

  /**
   * @return A climb object without hardware.
   */
  public static Climb none() {
    return new Climb(new NoClimb());
  }

  /**
   * The constructor of the climb subsystem.
   *
   * @param hardware our IO interface that represents the mechanism
   */
  private Climb(ClimbIO hardware) {
    setDefaultCommand(retract());

    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Meters));
    pid.reset(hardware.position());
    pid.setGoal(MIN_HEIGHT.in(Meters));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("elevator state", state.toString())),
            new SysIdRoutine.Mechanism(v -> hardware.setVoltage(v.in(Volts)), null, this));

    if (tuning) {
      SmartDashboard.putData(
          "Robot/elevator/quasistatic forward",
          sysIdRoutine
              .quasistatic(Direction.kForward)
              .until(() -> atPosition(MAX_HEIGHT.in(Meters)))
              .withName("elevator quasistatic forward"));
      SmartDashboard.putData(
          "Robot/elevator/quasistatic backward",
          sysIdRoutine
              .quasistatic(Direction.kReverse)
              .until(() -> atPosition(MIN_HEIGHT.in(Meters) + 0.1))
              .withName("elevator quasistatic backward"));
      SmartDashboard.putData(
          "Robot/elevator/dynamic forward",
          sysIdRoutine
              .dynamic(Direction.kForward)
              .until(() -> atPosition(MAX_HEIGHT.in(Meters)))
              .withName("elevator dynamic forward"));
      SmartDashboard.putData(
          "Robot/elevator/dynamic backward",
          sysIdRoutine
              .dynamic(Direction.kReverse)
              .until(() -> atPosition(MIN_HEIGHT.in(Meters) + 0.1))
              .withName("elevator dynamic backward"));
    }
  }

  /**
   * Checks if the position of the climb mechanism is within the error margins. The margin is {@link
   * #POSITION_TOLERANCE} centimeters.
   *
   * @param position The position of the climb mechanisms.
   * @return A boolean for if the climb is within the margins.
   */
  public boolean atPosition(double goal) {
    return Math.abs(goal - position()) < POSITION_TOLERANCE.in(Meters);
  }

  /**
   * @return The position of the climb mechanism in meters.
   */
  @Logged
  public double position() {
    return hardware.position();
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }

  /**
   * Calculates and sets the next voltage input to the motor.
   *
   * @param position The position to set the climb mechanism to.
   */
  private void update(double position) {
    double goal =
        Double.isNaN(position)
            ? MIN_HEIGHT.in(Meters)
            : MathUtil.clamp(position, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));
    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(hardware.position(), goal);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    hardware.setVoltage(feedforward + feedback);
  }

  /**
   * @return The position setpoint of the PID.
   */
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  @Override
  public void periodic() {
    setpoint.setLength(positionSetpoint());
    measurement.setLength(position());

    if (tuning) {
      ff.setKs(kS.get());
      ff.setKg(kG.get());
      ff.setKv(kV.get());
      ff.setKa(kA.get());
    }
  }

  /**
   * A command to move the climb to a certain height.
   *
   * @param height The height to set the climb to.
   * @return A command to move the climb.
   */
  public Command goTo(DoubleSupplier height) {
    return run(() -> update(height.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  /**
   * Wraps the {@link #goTo(DoubleSupplier height)} to be a double instead of a supplier.
   *
   * @param height The height to set the climb mechanism to.
   * @return A command to go to the height given.
   */
  public Command goTo(double height) {
    return goTo(() -> height);
  }

  /**
   * A command to retract the climb to the minimum height it can go to.
   *
   * @return A retracting command.
   */
  public Command retract() {
    return goTo(MIN_HEIGHT.in(Meters)).withName("retracting");
  }

  /**
   * A command to extend the climb to the max height it can go to.
   *
   * @return An extending command.
   */
  public Command extend() {
    return goTo(MAX_HEIGHT.in(Meters)).withName("extending");
  }
}
