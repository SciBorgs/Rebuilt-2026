package org.sciborgs1155.robot.commands.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

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
   * Calculates the direction and speed to run the shooter at to shoot accurately towards the goal.
   * This should take into account both the position of the shooter and the movement of the shooter.
   *
   * <p>If this specific method is not implemented in the shooting algorithm, this will just return
   * the same as calculate() without accel.
   *
   * @param pose The current field-relative position of the shooter. This is a Translation3d because
   *     the shooter may be offset from the center of the robot.
   * @param velocity The current translational velocity of the shooter.
   * @param accel The current translational acceleration of the shooter.
   * @return The direction and speed to run the shooter to shoot accurately towards the goal.
   */
  default Vector<N3> calculate(Translation3d pose, Vector<N2> velocity, Vector<N2> accel) {
    return calculate(pose, velocity);
  }
}
