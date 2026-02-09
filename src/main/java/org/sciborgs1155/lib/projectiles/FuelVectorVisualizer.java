package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.FieldConstants.fromPolarCoords;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import java.util.function.Supplier;

/** A modded {@code FuelVisualizer} to be compatible with vector input. */
public class FuelVectorVisualizer extends FuelVisualizer {
  /** A supplier for the launch velocity of the Fuel. */
  protected final Supplier<Vector<N3>> launchVelocityVector;

  /**
   * A modded {@code FuelVisualizer} to be compatible with vector input.
   *
   * @param launchVelocityVectorSupplier A supplier for the launch velocity of the Fuel.
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
   */
  public FuelVectorVisualizer(
      Supplier<Vector<N3>> launchVelocityVectorSupplier,
      Supplier<Pose3d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    super(
        RadiansPerSecond::zero,
        Radians::zero,
        Radians::zero,
        robotPoseSupplier,
        robotVelocitySupplier);
    launchVelocityVector = launchVelocityVectorSupplier;
  }

  @Override
  protected Vector<N3> launchVelocity(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return launchVelocityVector.get();
  }

  /**
   * Utility method for calculating velocity vector.
   *
   * @param robotPose The current pose of the robot.
   * @return The field-relative translation of the shooter.
   */
  public static Translation3d shooterTranslation(Pose3d robotPose) {
    return ROBOT_TO_SHOOTER.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  }

  /**
   * Utility method for calculating velocity vector. <br>
   * </br> NOTE: Rotational velocity calculation does not account for turret orientation.
   *
   * @param robotPose The current pose of the robot.
   * @param robotVelocity The robot-relative velocity of the robot.
   * @return The field-relative translation of the shooter.
   */
  public static Vector<N2> shooterVelocity(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    // SHOOTER ROTATIONAL VELOCITY
    Vector<N2> rotationalVelocity =
        fromPolarCoords(
            robotVelocity.omegaRadiansPerSecond * ROBOT_TO_SHOOTER.getNorm(),
            robotPose.toPose2d().getRotation().plus(Rotation2d.kCCW_90deg));

    return VecBuilder.fill(
        robotVelocity.vxMetersPerSecond + rotationalVelocity.get(X),
        robotVelocity.vyMetersPerSecond + rotationalVelocity.get(Y));
  }
}
