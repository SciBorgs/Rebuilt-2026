package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.*;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.*;
import static org.sciborgs1155.robot.hood.HoodConstants.HOOD_RADIUS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;

@SuppressWarnings("PMD.OneDeclarationPerLine")
public final class FuelVisualizer extends ProjectileVisualizer {
  private double scoreTolerance = Hub.INNER_WIDTH / 2;
  private double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);
  private double scoreDepth;

  private FuelVisualizer(
      Supplier<double[]> initialTranslation,
      Supplier<double[]> initialVelocity,
      Supplier<double[]> initialRotation,
      DoubleSupplier initialRotationalVelocity) {
    super(initialTranslation, initialVelocity, initialRotation, initialRotationalVelocity);
  }

  /** Creates a new visualizer from a supplier of launch parameters (speed, pitch, yaw). */
  public static FuelVisualizer fromLaunchParameters(
      Supplier<double[]> launchParameters, Drive drive) {
    DoubleSupplier heading = () -> drive.pose3d().getRotation().getZ();
    Supplier<double[]> shotVelocity = () -> robotRelativeShotVelocity(launchParameters.get());
    Supplier<ChassisSpeeds> chassisSpeeds = () -> drive.fieldRelativeChassisSpeeds();

    return new FuelVisualizer(
        () ->
            initialTranslation(
                fromTranslation(drive.pose3d().getTranslation()),
                shotVelocity.get(),
                heading.getAsDouble()),
        () ->
            initialVelocity(
                shotVelocity.get(),
                heading.getAsDouble(),
                chassisSpeeds.get().vxMetersPerSecond,
                chassisSpeeds.get().vyMetersPerSecond,
                chassisSpeeds.get().omegaRadiansPerSecond),
        () -> initialRotation(shotVelocity.get(), heading.getAsDouble()),
        () -> initialRotationalVelocity());
  }

  /** Creates a new visualizer from a supplier of launch parameters (speed, pitch, yaw). */
  public static FuelVisualizer fromLaunchParameters(
      DoubleSupplier speed, DoubleSupplier pitch, DoubleSupplier yaw, Drive drive) {
    return fromLaunchParameters(
        () -> new double[] {speed.getAsDouble(), pitch.getAsDouble(), yaw.getAsDouble()}, drive);
  }

  /**
   * Configures visualizer scoring parameters.
   *
   * @param goal the translation of the goal
   * @param tolerance the planar tolerance to be considered 'scored'
   * @param depth the vertical distance below the goal to check for scoring/missing
   */
  public FuelVisualizer withScoringParameters(double[] goal, double tolerance, double depth) {
    targetPose = goal.clone();
    scoreDepth = depth;
    scoreTolerance = tolerance;

    return this;
  }

  @Override
  protected Projectile createProjectile() {
    return new Fuel().withScoringParameters(targetPose, scoreTolerance, scoreDepth);
  }

  @Override
  public void updateLogging() {
    super.updateLogging();

    if (ending() == null) return;
    double deltaX = targetPose[X] - ending().getTranslation().getX();
    double deltaY = targetPose[Y] - ending().getTranslation().getY();
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    LoggingUtils.log("Projectile Visualizer/Distance From Goal", distance);
  }

  protected static double[] robotToShooter(double heading) {
    double cosHeading = Math.cos(heading);
    double sinHeading = Math.sin(heading);

    double[] robotRelativeShooterPose = fromTranslation(ROBOT_TO_SHOOTER);

    return new double[] {
      robotRelativeShooterPose[X] * cosHeading - robotRelativeShooterPose[Y] * sinHeading,
      robotRelativeShooterPose[X] * sinHeading + robotRelativeShooterPose[Y] * cosHeading,
      robotRelativeShooterPose[Z]
    };
  }

  protected static double[] shooterToInitial(double[] shotVelocity, double heading) {
    double pitch = Math.asin(shotVelocity[Z] / norm(shotVelocity));
    double hoodAngle = Math.PI / 2 - pitch;

    double x = HOOD_RADIUS.in(Meters) * Math.cos(hoodAngle);
    double z = HOOD_RADIUS.in(Meters) * Math.sin(hoodAngle);

    double yaw = Math.atan2(shotVelocity[Y], shotVelocity[X]);

    return fieldRelative(fieldRelative(new double[] {x, 0, z}, yaw), heading);
  }

  protected static double[] initialTranslation(
      double[] robotTranslation, double[] shotVelocity, double heading) {
    double[] robotToShooter = robotToShooter(heading);
    double[] shooterToInitial = shooterToInitial(shotVelocity, heading);

    return new double[] {
      robotToShooter[X] + robotTranslation[X] + shooterToInitial[X],
      robotToShooter[Y] + robotTranslation[Y] + shooterToInitial[Y],
      robotToShooter[Z] + robotTranslation[Z] + shooterToInitial[Z]
    };
  }

  protected static double[] initialVelocity(
      double[] shotVelocity, double heading, double robotVx, double robotVy, double robotOmega) {
    double[] fieldRelative = fieldRelative(shotVelocity, heading);
    double[] shooterVelocity = shooterVelocity(robotVx, robotVy, robotOmega, heading);

    return new double[] {
      fieldRelative[X] + shooterVelocity[X],
      fieldRelative[Y] + shooterVelocity[Y],
      fieldRelative[Z] + shooterVelocity[Z]
    };
  }

  protected static double[] initialRotation(double[] shotVelocity, double heading) {
    double[] fieldRelative = fieldRelative(shotVelocity, heading);
    return new double[] {0, 0, Math.atan2(fieldRelative[Y], fieldRelative[X])};
  }

  protected static double initialRotationalVelocity() {
    return 0;
  }

  protected static double[] shooterVelocity(
      double robotVx, double robotVy, double robotOmega, double heading) {
    double[] robotToShooter = robotToShooter(heading);

    double shooterVx = robotVx - robotOmega * robotToShooter[Y];
    double shooterVy = robotVy + robotOmega * robotToShooter[X];

    return new double[] {shooterVx, shooterVy, 0};
  }

  protected static double[] robotRelativeShotVelocity(double[] launchParameters) {
    double pitch = launchParameters[PITCH];
    double yaw = launchParameters[YAW];

    double cosPitch = Math.cos(pitch);
    double sinPitch = Math.sin(pitch);
    double cosYaw = Math.cos(yaw);
    double sinYaw = Math.sin(yaw);

    return new double[] {
      cosPitch * cosYaw * launchParameters[SPEED],
      cosPitch * sinYaw * launchParameters[SPEED],
      sinPitch * launchParameters[SPEED]
    };
  }

  protected static double[] launchParameters(double[] shotVelocity) {
    double speed = norm(shotVelocity);
    if (speed < EPS) return new double[4];

    double yaw = Math.atan2(shotVelocity[Y] / speed, shotVelocity[X] / speed);
    double pitch = Math.asin(shotVelocity[Z] / speed);

    return new double[] {speed, pitch, yaw};
  }

  protected static double[] fieldRelative(double[] robotRelative, double heading) {
    double fieldRelativeX =
        robotRelative[X] * Math.cos(heading) - robotRelative[Y] * Math.sin(heading);
    double fieldRelativeY =
        robotRelative[X] * Math.sin(heading) + robotRelative[Y] * Math.cos(heading);

    return new double[] {fieldRelativeX, fieldRelativeY, robotRelative[Z]};
  }

  protected static double[] robotRelative(double[] fieldRelative, double heading) {
    double fieldRelativeX =
        fieldRelative[X] * Math.cos(-heading) - fieldRelative[Y] * Math.sin(-heading);
    double fieldRelativeY =
        fieldRelative[X] * Math.sin(-heading) + fieldRelative[Y] * Math.cos(-heading);

    return new double[] {fieldRelativeX, fieldRelativeY, fieldRelative[Z]};
  }

  protected static class Fuel extends Projectile {
    protected double scoreDepth;
    protected double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);

    protected double scoreRadius = Hub.INNER_WIDTH + FUEL_RADIUS;
    protected double scoreRadiusSq = scoreRadius * scoreRadius;

    protected boolean inScoringPlane, inScoringRadius;

    protected final double[] drag = new double[3];
    protected final double[] lift = new double[3];

    protected Fuel withScoringParameters(double[] goal, double tolerance, double depth) {
      targetPose = goal.clone();
      scoreDepth = depth;
      scoreRadius = tolerance + FUEL_RADIUS;
      scoreRadiusSq = scoreRadius * scoreRadius;

      return this;
    }

    @Override
    protected double[] drag() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
      double scale = -DRAG_CONSTANT * Math.sqrt(vx * vx + vy * vy + vz * vz);

      drag[X] = vx * scale;
      drag[Y] = vy * scale;
      drag[Z] = vz * scale;

      return drag.clone();
    }

    @Override
    protected double[] lift() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
      double scale = LIFT_CONSTANT * omega;

      lift[X] = scale * -vx;
      lift[Y] = 0;
      lift[Z] = scale * vz;

      return lift.clone();
    }

    @Override
    protected double torque() {
      return 0;
    }

    @Override
    protected void periodic() {
      super.periodic();

      double deltaX = x - targetPose[X];
      double deltaY = y - targetPose[Y];

      double deltaPlanarSq = deltaX * deltaX + deltaY * deltaY;
      double deltaZ = z - targetPose[Z] + scoreDepth;

      inScoringPlane = deltaZ >= 0 && deltaZ < SCORE_WINDOW && vz < 0;
      inScoringRadius = deltaPlanarSq <= scoreRadiusSq;
    }

    @Override
    protected boolean willScore() {
      return inScoringPlane && inScoringRadius;
    }

    @Override
    protected boolean willMiss() {
      return inScoringPlane && !inScoringRadius || z < FUEL_RADIUS;
    }

    @Override
    protected void reset() {
      super.reset();

      inScoringPlane = false;
      inScoringRadius = false;
    }
  }
}
