package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

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

  private double lastAngularVelocity;
  private double alpha;

  /** Creates a new NavXGyro and registers it with FaultLogger. */
  public NavXGyro() {
    FaultLogger.register(ahrs);

    position = OdometryThread.getInstance().registerSignal(ahrs::getYaw);
    timestamp = OdometryThread.getInstance().makeTimestampQueue();

    lastAngularVelocity = 0;
    alpha = 0;
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
    Drive.LOCK.lock();
    try {
      double[][] data = {
        position.stream().mapToDouble((Double d) -> d).toArray(),
        timestamp.stream().mapToDouble((Double d) -> d).toArray()
      };
      position.clear();
      timestamp.clear();
      return data;
    } finally {
      Drive.LOCK.unlock();
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
  public double alpha() {
    return alpha;
  }

  @Override
  public void reset(Rotation2d heading) {
    ahrs.setAngleAdjustment(heading.getDegrees());
    ahrs.reset();
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void periodic() {
    alpha = (ahrs.getRate() - lastAngularVelocity) / PERIOD.in(Seconds);
    lastAngularVelocity = ahrs.getRate();
  }
}
