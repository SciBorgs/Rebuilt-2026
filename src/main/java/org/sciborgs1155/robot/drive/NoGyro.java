package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N2;

/** GyroIO implementation for nonexistent gyro */
public class NoGyro implements GyroIO {
  private final Rotation3d rotation = new Rotation3d();

  @Override
  public void close() throws Exception {}

  @Override
  public double rate() {
    return 0;
  }

  @Override
  public Rotation3d rotation3d() {
    return rotation;
  }

  @Override
  public double[][] odometryData() {
    return new double[20][20];
  }

  @Override
  public Vector<N2> acceleration() {
    return VecBuilder.fill(0, 0);
  }

  @Override
  public void reset(Rotation2d heading) {}
}
