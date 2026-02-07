package org.sciborgs1155.lib.shooting;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;

import java.util.function.DoubleBinaryOperator;

import org.sciborgs1155.lib.LoggingUtils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooting implements ShootingAlgorithm {

  // constants
  final double launchSpeed = 10;

  @Override
  public Vector<N3> calculate(Translation3d pose, Vector<N2> velocity) {
    // target vector (from robot to goal not moving)
    Vector<N2> robotToTarget = BLUE_HUB.toTranslation2d().toVector().minus(pose.toTranslation2d().toVector());

    Vector<N2> directShotVelocity = robotToTarget.unit().times(launchSpeed);
  
    Vector<N2> shot = directShotVelocity.minus(velocity);

    // horizontal angle to shoot at goal
    double angle = Math.atan2(shot.get(1), shot.get(0));

    double d = robotToTarget.norm();
    double H = BLUE_HUB.getZ() - pose.getZ();
    double G = -9.81;
    double V = launchSpeed;

    double V2 = V * V;
    double V4 = pow(V, 4);
    double d2 = pow(d, 2);

    double radicand = V4 - (G * (G * d2 + 2 * H * V2));
    double pitch = -atan((V2 + sqrt(radicand)) / (G * d));

    // final X, Y, and Z
    double horizontalVelocity = Math.cos(pitch) * launchSpeed;
    double finalX = Math.cos(angle) * horizontalVelocity;
    double finalY = Math.sin(angle) * horizontalVelocity;
    double finalZ = Math.sin(pitch) * launchSpeed;

    LoggingUtils.log("radicand", radicand);
    LoggingUtils.log("pitch", pitch * 180 / PI);
    LoggingUtils.log("finalz", finalZ);

    return VecBuilder.fill(finalX, finalY, finalZ);
  }
}