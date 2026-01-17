package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import org.sciborgs1155.robot.FieldConstants;

public class FuelVisualizer {
  private static final double GRAVITY = -9.81;
  private static final double MASS = 0.2;
  private static final double DRAG_CONSTANT = 1;

  public ArrayList<Pose3d> generateTrajectory(Pose3d initialPose, Transform3d initialVelocity, int resolution) {
    Pose3d position = initialPose;
    Transform3d velocity = initialVelocity.div(resolution);
    ArrayList<Pose3d> trajectory = new ArrayList<>();

    Transform3d acceleration = new Transform3d();
    Transform3d gravity = new Transform3d(0, GRAVITY, 0, Rotation3d.kZero).div(resolution);
    Transform3d drag = velocity.inverse().times(DRAG_CONSTANT / MASS);

    while (FieldConstants.inField(position)) {
      position = position.plus(velocity);
      drag = velocity.inverse().times(DRAG_CONSTANT / MASS);
      acceleration = gravity.plus(drag);
      velocity = velocity.plus(acceleration);
    }

    return trajectory;
  }
}
