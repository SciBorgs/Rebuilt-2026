package org.sciborgs1155.lib.shooting;

import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;
import org.sciborgs1155.lib.LoggingUtils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooting implements ShootingAlgorithm {

  // constants
  final double launchSpeed = 10;

  @Override
  public Vector<N3> calculate(Translation3d pose, Vector<N2> velocity) {
    // target vector (from robot to goal not moving)
    Vector<N2> robotToTarget = BLUE_HUB.toTranslation2d().minus(pose.toTranslation2d()).toVector();

    double d = robotToTarget.norm();
    double H = BLUE_HUB.getZ() - pose.getZ() + 0.2;
    double G = 9.81;
    double V = launchSpeed;

    double V2 = pow(V, 2);
    double V4 = pow(V, 4);
    double d2 = pow(d, 2);

    double radicand = V4 - (G * (G * d2 + (2 * H * V2)));
    double pitch = atan2((V2 + sqrt(radicand)) , (G * d));

    // final X, Y, and Z
    double horizontalVelocity = Math.cos(pitch) * launchSpeed;
    double finalX = robotToTarget.unit().get(0) * horizontalVelocity;
    double finalY = robotToTarget.unit().get(1) * horizontalVelocity;
    double finalZ = Math.sin(pitch) * launchSpeed;

    LoggingUtils.log("radicand", radicand);
    LoggingUtils.log("distance to hub", d);
    LoggingUtils.log("pitch", pitch * 180 / PI);
    LoggingUtils.log("finalz", finalZ);
    LoggingUtils.log("Height", H);

    Vector<N3> directShotVelocity = VecBuilder.fill(finalX, finalY, finalZ);
    Vector<N3> shotVelocity = directShotVelocity.minus(VecBuilder.fill(velocity.get(0), velocity.get(1), 0));

    return shotVelocity;
  }
}