package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
public class Climb extends SubsystemBase implements AutoCloseable {
  private final ClimbIO hardware;
  private final SysIdRoutine sysIdRoutine;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV);

  private final ClimbVisualizer setpoint =
      new ClimbVisualizer("setpoint visualizer", new Color8Bit(0, 0, 255));

  private final ClimbVisualizer measurement =
      new ClimbVisualizer("measurement visualizer", new Color8Bit(255, 0, 0));

  private final DoubleEntry S = Tuning.entry("/Robot/tuning/elevator/kS", kS);
  private final DoubleEntry G = Tuning.entry("/Robot/tuning/elevator/kG", kG);
  private final DoubleEntry V = Tuning.entry("/Robot/tuning/elevator/kV", kV);
  private final DoubleEntry A = Tuning.entry("/Robot/tuning/elevator/kA", kA);

  public Climb(ClimbIO hardware) {
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

    if (TUNING) {
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

  public boolean atPosition(double position) {
    // given position - |actual position| < tolerance
    return Meters.of(position).minus(Meters.of(position())).magnitude()
        < POSITION_TOLERANCE.in(Meters);
  }

  public double position() {
    return hardware.position();
  }

  public static Climb create() {
    return new Climb(Robot.isReal() ? new RealClimb() : new SimClimb());
  }

  public static Climb none() {
    return new Climb(new NoClimb());
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }

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

  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  public void periodic() {
    setpoint.setLength(positionSetpoint());
    measurement.setLength(position());

    if (TUNING) {
      ff.setKs(S.get());
      ff.setKg(G.get());
      ff.setKv(V.get());
      ff.setKa(A.get());
    }
  }

  public Command goTo(DoubleSupplier height) {
    return run(() -> update(height.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  public Command goTo(double height) {
    return goTo(() -> height);
  }

  public Command retract() {
    return goTo(MIN_HEIGHT.in(Meters)).withName("retracting");
  }

  public Command extend() {
    return goTo(MAX_HEIGHT.in(Meters)).withName("extending");
  }
}
