package org.sciborgs1155.robot.commands.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@FunctionalInterface
public interface ShootingAlgorithm {

  /**
   * Calculates the direction and speed to run the shooter at to shoot accurately towards the goal.
   * This should take into account both the position of the shooter and the movement of the shooter.
   *
   * @param pose The current field-relative position of the shooter. This is a Translation3d because
   *     the shooter may be offset from the center of the robot.
   * @param velocity The current translational velocity of the shooter.
   * @return The direction and speed to run the shooter to shoot accurately towards the goal.
   */
  Vector<N3> calculate(Translation3d pose, Vector<N2> velocity);

  /**
   * Converts a shooting algorithm output to a shot velocity vector (X, Y, and Z) which is
   * compatible with visualizers.
   *
   * @param robotPose A supplier for the pose of the drivetrain.
   * @param robotVelocity A supplier for the velocity of the drivetrain.
   * @return A double[] supplier that can be passed into the constructor of a visualizer.
   */
  static Supplier<double[]> toShotVelocitySupplier(
      ShootingAlgorithm shootingAlgorithm,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    return () ->
        FuelVisualizer.shotVelocity(shootingAlgorithm, robotPose.get(), robotVelocity.get());
  }

  /**
   * Converts shooter properties to a shot velocity vector (X, Y, and Z) which is compatible with
   * visualizers.
   *
   * @param speed A supplier for the launch speed of the FUEL.
   * @param pitch A supplier for the pitch of the shooter.
   * @param yaw A supplier for the yaw of the shooter.
   * @param robotPose A supplier for the pose of the drivetrain.
   * @return A double[] supplier that can be passed into the constructor of a visualizer.
   */
  static Supplier<double[]> toShotVelocitySupplier(
      DoubleSupplier speed, DoubleSupplier pitch, DoubleSupplier yaw, Supplier<Pose3d> robotPose) {
    return () ->
        FuelVisualizer.shotVelocity(
            speed.getAsDouble(), pitch.getAsDouble(), yaw.getAsDouble(), robotPose.get());
  }
}
