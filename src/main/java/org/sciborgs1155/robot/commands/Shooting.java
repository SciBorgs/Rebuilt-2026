package org.sciborgs1155.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {
  private final Shooter shooter;
  private final Drive drive;

  /**
   * Sets the subsystems to be used for shooting commands.
   *
   * @param shooter The shooter subsystem.
   * @param drive The drive subsystem.
   */
  public Shooting(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
  }

  /**
   * Runs the shooter before feeding it fuel.
   *
   * @param desiredVelocity The velocity in radians per second to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(AngularVelocity desiredVelocity) {
    return null;
  }

  /**
   * Runs shooter to desired velocity, runs feeder once it reaches its velocity and shootCondition
   * is true.
   *
   * @param desiredVelocity Target velocity for the flywheel.
   * @param shootCondition Condition after which the feeder will run.
   */
  public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
    return null;
  }
}
