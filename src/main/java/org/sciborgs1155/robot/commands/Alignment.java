package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.lib.LoggingUtils.log;
import static org.sciborgs1155.robot.FieldConstants.allianceFromPose;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.Supplier;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.RepulsorFieldPlanner;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;

public class Alignment {
  @NotLogged private final Drive drive;

  private final RepulsorFieldPlanner planner = new RepulsorFieldPlanner();

  private final Fault alternateAlliancePathfinding =
      new Fault(
          "Alternate Alliance Pathfinding",
          "The robot is attempting to pathfind to a pose on the other alliance.",
          FaultType.WARNING);

  /**
   * Constructor for an Alignment command object.
   *
   * @param drive The operated drivetrain.
   * @param elevator The operated elevator.
   */
  public Alignment(Drive drive) {
    this.drive = drive;
  }

  /**
   * Moves the robot relative to the current pose, given a transform.
   *
   * @param transform A transform to move by.
   * @return A Command that drives to the transformed pose.
   */
  public Command moveRobotRelative(Transform2d transform) {
    return Commands.defer(
        () -> {
          Pose2d goal = drive.pose().transformBy(transform);
          return drive.driveTo(goal);
        },
        Set.of(drive));
  }

  /**
   * Aligns to a supplied goal pose.
   *
   * @param goal The field pose to align to.
   * @return A Command to pathfind align to the pose.
   */
  public Command alignTo(Supplier<Pose2d> goal) {
    return Commands.runOnce(() -> log("/Robot/alignment/goal pose", goal.get(), Pose2d.struct))
        .andThen(
            pathfind(goal, Meters.of(1))
                .asProxy()
                .andThen(drive.driveTo(goal).asProxy())
                .onlyWhile(
                    () ->
                        !FaultLogger.report(
                            allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                            alternateAlliancePathfinding)));
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @param maxSpeed The maximum speed the path will command the drivetrain to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal, LinearVelocity maxSpeed, Distance tolerance) {
    double speed = maxSpeed.in(MetersPerSecond);
    return drive
        .run(
            () -> {
              Tracer.startTrace("repulsor pathfinding");
              planner.setGoal(goal.get().getTranslation());
              drive.goToSample(
                  planner.getCmd(drive.pose(), drive.fieldRelativeChassisSpeeds(), speed, true),
                  goal.get().getRotation());
              Tracer.endTrace();
            })
        .until(() -> drive.atTranslation(goal.get().getTranslation(), tolerance))
        .onlyWhile(
            () ->
                !FaultLogger.report(
                    allianceFromPose(goal.get()) != allianceFromPose(drive.pose()),
                    alternateAlliancePathfinding))
        .withName("pathfind");
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal, Distance tolerance) {
    return pathfind(goal, DriveConstants.MAX_SPEED.times(0.7), tolerance);
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal, LinearVelocity maxSpeed) {
    return pathfind(goal, maxSpeed, Translation.TOLERANCE);
  }

  /**
   * Pathfinds around obstacles and drives to a certain pose on the field.
   *
   * @param goal The field pose to pathfind to.
   * @return A Command to pathfind to an onfield pose.
   */
  public Command pathfind(Supplier<Pose2d> goal) {
    return pathfind(goal, Translation.TOLERANCE);
  }

  /**
   * Warms up the pathfind command by telling drive to drive to itself.
   *
   * @return A warmup command that pathfinds to the robot's current position.
   */
  @SuppressWarnings("PMD.SystemPrintln") // Please replace with better logging
  public Command warmupCommand() {
    return pathfind(() -> drive.pose(), MetersPerSecond.of(0))
        .withTimeout(3)
        .andThen(() -> System.out.println("[Alignment] Finished warmup"))
        .ignoringDisable(true);
  }
}
