package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

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
   * Converts a shooting algorithm output to a double[] of launch velocity (X, Y, and Z) which is compatible with visualizers.
   * 
   * @param robotPose The pose of the drivetrain.
   * @param robotVelocity The velocity of the drivetrain.
   * @return A double[] that can be passed into the constructor of a visualizer.
   */
  default double[] calculateToDoubleArray(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return calculate(robotPose.getTranslation(), VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)).getData();
  }
}
