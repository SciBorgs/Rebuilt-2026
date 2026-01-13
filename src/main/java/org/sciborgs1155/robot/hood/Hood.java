package org.sciborgs1155.robot.hood;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.sciborgs1155.robot.Robot;

public class Hood extends SubsystemBase {

  private final HoodIO hardware;

  private final ProfiledPIDController fb =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));

  /** Arm feed forward controller. */
  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  /** Routine for recording and analyzing motor data. */
  private final SysIdRoutine sysIdRoutine;

  public Hood(HoodIO hardware) {
    this.hardware = hardware;
  }

  public static Hood create() {
    return new Hood(Robot.isReal() ? new RealHood() : new SimHood());
  }

  public static Hood none() {
    return new Hood(new NoHood());
  }
}
