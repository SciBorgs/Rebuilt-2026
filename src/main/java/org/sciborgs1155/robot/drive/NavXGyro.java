package org.sciborgs1155.robot.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N2;
import java.util.Queue;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for NavX */
public class NavXGyro implements GyroIO {
  private final AHRS ahrs = new AHRS(NavXComType.kMXP_SPI);

  private final Queue<Double> position;
  private final Queue<Double> timestamp;

  public NavXGyro() {
    FaultLogger.register(ahrs);

    position = OdometryThread.getInstance().registerSignal(ahrs::getYaw);
    timestamp = OdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public double rate() {
    return ahrs.getRate();
  }

  @Override
  public Rotation3d rotation3d() {
    return ahrs.getRotation3d();
  }

  @Override
  public double[][] odometryData() {
    Drive.lock.lock();
    try {
      double[][] data = {
        position.stream().mapToDouble((Double d) -> d).toArray(),
        timestamp.stream().mapToDouble((Double d) -> d).toArray()
      };
      position.clear();
      timestamp.clear();
      return data;
    } finally {
      Drive.lock.unlock();
    }
  }

  @Override
  public Vector<N2> acceleration() {
    return VecBuilder.fill(
        ahrs.getWorldLinearAccelX(),
        ahrs.getWorldLinearAccelY()); // .rotateBy(canandgyro.getRotation2d());

    // TODO We don't know if this is field relative or robot relative. if robot relative add in the
    // commented code.
  }

  @Override
  public void reset(Rotation2d heading) {
    ahrs.setAngleAdjustment(heading.getDegrees());
    ahrs.reset();
  }

  @Override
  public void close() throws Exception {}
}
