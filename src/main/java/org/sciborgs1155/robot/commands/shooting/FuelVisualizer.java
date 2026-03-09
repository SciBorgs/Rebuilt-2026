package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.Constants.Robot.ROBOT_TO_SHOOTER;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;

@SuppressWarnings("PMD.AvoidLiteralsInIfCondition")
public class FuelVisualizer extends ProjectileVisualizer {
  private double scoreTolerance = Hub.INNER_WIDTH / 2;
  private double scoreDepth;
  private double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);

  protected static final double FUEL_MASS = 0.225;
  protected static final double FUEL_RADIUS = 0.075;

  protected static final double AIR_DENSITY = 1.225;
  protected static final double[] GRAVITY = new double[] {0, 0, -9.80665};

  /** Multiplied by velocity squared to compute drag acceleration. */
  private static final double DRAG_CONSTANT =
      0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS / FUEL_MASS;

  /** Multiplied by spin and velocity to produce magnus acceleration. */
  private static final double LIFT_CONSTANT =
      4.0 / 3.0 * Math.PI * FUEL_RADIUS * FUEL_RADIUS * FUEL_RADIUS * AIR_DENSITY / FUEL_MASS;

  private static final double SCORE_WINDOW = FUEL_RADIUS / 2;

  protected FuelVisualizer(Supplier<double[]> initialVelocity, Supplier<Pose3d> robotPose) {
    super(
        () -> initialTranslation(initialVelocity.get(), robotPose.get()),
        () -> initialVelocity.get(),
        () -> initialRotation(initialVelocity.get()),
        () -> initialRotationalVelocity());
  }

  public static FuelVisualizer fromLaunchParameters(
      Supplier<double[]> launchParameters, Drive drive) {
    return new FuelVisualizer(
        () ->
            initialVelocity(
                shotVelocity(launchParameters.get(), drive.pose3d().getRotation().getZ()),
                drive.pose3d(),
                drive.fieldRelativeChassisSpeeds()),
        drive::pose3d);
  }

  public static FuelVisualizer fromLaunchParameters(
      DoubleSupplier speed, DoubleSupplier pitch, DoubleSupplier yaw, Drive drive) {
    return fromLaunchParameters(
        () -> new double[] {0, speed.getAsDouble(), pitch.getAsDouble(), yaw.getAsDouble()}, drive);
  }

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
    LoggingUtils.log(
        "Projectile Visualizer/Distance From Goal",
        Math.hypot(
            targetPose[X] - ending().getTranslation().getX(),
            targetPose[Y] - ending().getTranslation().getY()));
  }

  protected static double[] shooterPose(Pose3d robotPose) {
    double heading = robotPose.getRotation().getZ();
    double cosHeading = Math.cos(heading);
    double sinHeading = Math.sin(heading);

    double[] robotRelativeShooterPose = fromTranslation(ROBOT_TO_SHOOTER);

    return new double[] {
      robotRelativeShooterPose[X] * cosHeading - robotRelativeShooterPose[Y] * sinHeading,
      robotRelativeShooterPose[X] * sinHeading + robotRelativeShooterPose[Y] * cosHeading,
      robotRelativeShooterPose[Z]
    };
  }

  protected static double[] initialTranslation(double[] shotVelocity, Pose3d robotPose) {
    double[] robotToShooter = shooterPose(robotPose);

    return new double[] {
      robotToShooter[X] + robotPose.getX(),
      robotToShooter[Y] + robotPose.getY(),
      robotToShooter[Z] + robotPose.getZ()
    };
  }

  protected static double[] initialVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double[] robotToShooter = shooterPose(robotPose);

    double radius = Math.hypot(robotToShooter[X], robotToShooter[Y]);
    double tangentialSpeed = robotVelocity.omegaRadiansPerSecond * radius;
    double tangentialDirection = robotPose.getRotation().getZ() + Math.PI / 2.0;

    double shooterVx =
        robotVelocity.vxMetersPerSecond + tangentialSpeed * Math.cos(tangentialDirection);
    double shooterVy =
        robotVelocity.vyMetersPerSecond + tangentialSpeed * Math.sin(tangentialDirection);

    return new double[] {shotVelocity[X] + shooterVx, shotVelocity[Y] + shooterVy, shotVelocity[Z]};
  }

  protected static double[] initialRotation(double[] shotVelocity) {
    return new double[] {0, 0, Math.atan2(shotVelocity[Y], shotVelocity[X])};
  }

  protected static double initialRotationalVelocity() {
    return 0;
  }

  public static class Fuel extends Projectile {
    protected double scoreDepth;
    protected double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);

    protected double scoreRadius = Hub.INNER_WIDTH + FUEL_RADIUS;
    protected double scoreRadiusSq = scoreRadius * scoreRadius;

    protected boolean inScoringPlane = false;
    protected boolean inScoringRadius = false;

    protected final double[] drag = new double[3];

    public Fuel withScoringParameters(double[] goal, double tolerance, double depth) {
      targetPose = goal.clone();
      scoreDepth = depth;
      scoreRadius = tolerance + FUEL_RADIUS;
      scoreRadiusSq = scoreRadius * scoreRadius;

      return this;
    }

    @Override
    protected double[] weight() {
      return GRAVITY;
    }

    @Override
    protected double[] drag() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
      double scale = -DRAG_CONSTANT * Math.sqrt(vx * vx + vy * vy + vz * vz);
      return new double[] {vx * scale, vy * scale, vz * scale};
    }

    @Override
    protected double[] lift() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
      return new double[] {LIFT_CONSTANT * (-omega * vx), 0, LIFT_CONSTANT * (omega * vz)};
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

  protected static double[] shotVelocity(double[] launchParameters, double heading) {
    double pitch = launchParameters[PITCH];
    double yaw = launchParameters[YAW];

    double cosPitch = Math.cos(pitch);
    double sinPitch = Math.sin(pitch);
    double cosYaw = Math.cos(yaw);
    double sinYaw = Math.sin(yaw);

    double directionX = cosPitch * cosYaw;
    double directionY = cosPitch * sinYaw;

    double cosHeading = Math.cos(heading);
    double sinHeading = Math.sin(heading);

    return new double[] {
      (directionX * cosHeading - directionY * sinHeading) * launchParameters[SPEED],
      (directionX * sinHeading + directionY * cosHeading) * launchParameters[SPEED],
      sinPitch * launchParameters[SPEED]
    };
  }

  protected static double[] launchParameters(double[] shotVelocity, double heading) {
    double speed = norm(shotVelocity);
    if (speed < 1e-6) return new double[4];

    double yaw = Math.atan2(shotVelocity[Y] / speed, shotVelocity[X] / speed) - heading;
    double pitch = Math.asin(shotVelocity[Z] / speed);

    return new double[] {0, speed, pitch, yaw};
  }

  protected static double[] fromTranslation(Translation3d translation) {
    return new double[] {translation.getX(), translation.getY(), translation.getZ()};
  }

  protected static double norm(double[] vector) {
    double x = vector[X];
    double y = vector[Y];
    double z = vector[Z];

    return Math.sqrt(x * x + y * y + z * z);
  }
}
