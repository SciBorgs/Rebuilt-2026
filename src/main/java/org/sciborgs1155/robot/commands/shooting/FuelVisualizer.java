package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.*;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.*;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;

import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.drive.Drive;

@SuppressWarnings("PMD.OneDeclarationPerLine")
public final class FuelVisualizer extends ProjectileVisualizer {
  private double scoreTolerance = Hub.INNER_WIDTH / 2;
  private final double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);
  private double scoreDepth;

  private final DoubleSupplier speed, pitch, yaw;
  private final DoubleSupplier robotVx, robotVy, robotOmega;
  private final DoubleSupplier robotX, robotY, heading;

  private final CachedVector initialTranslation, initialVelocity, initialRotation;

  private FuelVisualizer(
      DoubleSupplier speed,
      DoubleSupplier pitch,
      DoubleSupplier yaw,
      DoubleSupplier robotX,
      DoubleSupplier robotY,
      DoubleSupplier heading,
      DoubleSupplier vx,
      DoubleSupplier vy,
      DoubleSupplier robotOmega,
      CachedVector initialTranslation,
      CachedVector initialVelocity,
      CachedVector initialRotation,
      DoubleSupplier initialRotationalVelocity) {
    super(
        initialTranslation.x(),
        initialTranslation.y(),
        initialTranslation.z(),
        initialVelocity.x(),
        initialVelocity.y(),
        initialVelocity.z(),
        initialRotation.x(),
        initialRotation.y(),
        initialRotation.z(),
        initialRotationalVelocity);

    this.speed = speed;
    this.pitch = pitch;
    this.yaw = yaw;
    this.robotVx = vx;
    this.robotVy = vy;
    this.robotOmega = robotOmega;
    this.heading = heading;
    this.robotX = robotX;
    this.robotY = robotY;
    
    this.initialTranslation = initialTranslation;
    this.initialVelocity = initialVelocity;
    this.initialRotation = initialRotation;
  }

  protected static FuelVisualizer fromLaunchParameters(
      DoubleSupplier speed, DoubleSupplier pitch, DoubleSupplier yaw, Drive drive) {
    DoubleSupplier robotX = () -> drive.pose().getX();
    DoubleSupplier robotY = () -> drive.pose().getY();
    DoubleSupplier heading = () -> drive.heading().getRadians();
    DoubleSupplier robotVx = () -> drive.fieldRelativeChassisSpeeds().vxMetersPerSecond;
    DoubleSupplier robotVy = () -> drive.fieldRelativeChassisSpeeds().vyMetersPerSecond;
    DoubleSupplier robotOmega = () -> drive.fieldRelativeChassisSpeeds().omegaRadiansPerSecond;

    CachedVector initialTranslation =
        new CachedVector(
            () ->
                initialTranslation(
                    robotX.getAsDouble(),
                    robotY.getAsDouble(),
                    pitch.getAsDouble(),
                    yaw.getAsDouble(),
                    heading.getAsDouble()));

    CachedVector initialVelocity =
        new CachedVector(
            () ->
                initialVelocity(
                    speed.getAsDouble(),
                    pitch.getAsDouble(),
                    yaw.getAsDouble(),
                    heading.getAsDouble(),
                    robotVx.getAsDouble(),
                    robotVy.getAsDouble(),
                    robotOmega.getAsDouble(),
                    0));

    CachedVector initialRotation =
        new CachedVector(() -> initialRotation(yaw.getAsDouble(), heading.getAsDouble()));

    return new FuelVisualizer(
        speed,
        pitch,
        yaw,
        robotX,
        robotY,
        heading,
        robotVx,
        robotVy,
        robotOmega,
        initialTranslation,
        initialVelocity,
        initialRotation,
        FuelVisualizer::initialRotationalVelocity);
  }

  /**
   * Configures visualizer scoring parameters.
   *
   * @param goal the translation of the goal
   * @param tolerance the planar tolerance to be considered 'scored'
   * @param depth the vertical distance below the goal to check for scoring/missing
   */
  public FuelVisualizer withScoringParameters(double[] goal, double tolerance, double depth) {
    System.arraycopy(goal, 0, targetPose, 0, 3);
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

    initialTranslation.invalidate();
    initialVelocity.invalidate();
    initialRotation.invalidate();

    if (ending() == null) return;
    double deltaX = targetPose[X] - ending().getTranslation().getX();
    double deltaY = targetPose[Y] - ending().getTranslation().getY();
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    LoggingUtils.log("Projectile Visualizer/Distance From Goal", distance);

    LoggingUtils.log("Projectile Visualizer/Launch Data/SPEED", speed.getAsDouble());
    LoggingUtils.log("Projectile Visualizer/Launch Data/PITCH", pitch.getAsDouble());

    LoggingUtils.log("Projectile Visualizer/Launch Data/YAW", yaw.getAsDouble());

    LoggingUtils.log("Projectile Visualizer/Launch Data/ROBOT X", robotX.getAsDouble());
    LoggingUtils.log("Projectile Visualizer/Launch Data/ROBOT Y", robotY.getAsDouble());
    LoggingUtils.log("Projectile Visualizer/Launch Data/HEADING", heading.getAsDouble());

    LoggingUtils.log("Projectile Visualizer/Launch Data/ROBOT VX", robotVx.getAsDouble());
    LoggingUtils.log("Projectile Visualizer/Launch Data/ROBOT VY", robotVy.getAsDouble());
    LoggingUtils.log("Projectile Visualizer/Launch Data/ROBOT OMEGA", robotOmega.getAsDouble());
  }

  protected static double[] robotToShooter(double heading) {
    double cosHeading = Math.cos(heading);
    double sinHeading = Math.sin(heading);

    return new double[] {
      ROBOT_TO_SHOOTER[X] * cosHeading - ROBOT_TO_SHOOTER[Y] * sinHeading,
      ROBOT_TO_SHOOTER[X] * sinHeading + ROBOT_TO_SHOOTER[Y] * cosHeading,
      ROBOT_TO_SHOOTER[Z]
    };
  }

  protected static double[] shooterToInitial(double pitch, double yaw, double heading) {
    double angle = Math.PI / 2 - pitch + MIN_ANGLE.in(Radians);
    double horizontal = SHOOTER_TO_FLYWHEEL[X] - SHOOTER_RADIUS * Math.cos(angle);
    double vertical = SHOOTER_TO_FLYWHEEL[Z] + SHOOTER_RADIUS * Math.sin(angle);

    return fieldRelative(new double[] {horizontal, 0, vertical}, yaw + heading);
  }

  protected static double[] initialTranslation(
      double robotX, double robotY, double pitch, double yaw, double heading) {
    double[] robotToShooter = robotToShooter(heading);
    double[] shooterToInitial = shooterToInitial(pitch, yaw, heading);

    return new double[] {
      robotToShooter[X] + robotX + shooterToInitial[X],
      robotToShooter[Y] + robotY + shooterToInitial[Y],
      robotToShooter[Z] + shooterToInitial[Z]
    };
  }

  protected static double[] initialVelocity(
      double speed,
      double pitch,
      double yaw,
      double heading,
      double robotVx,
      double robotVy,
      double robotOmega,
      double yawOmega) {
    double[] launchParameters = {speed, pitch, yaw};
    double[] fieldRelative = fieldRelative(robotRelativeShotVelocity(launchParameters), heading);
    double[] shooterVelocity = shooterVelocity(robotVx, robotVy, robotOmega, heading);
    double[] shooterToInitial = shooterToInitial(pitch, yaw, heading);

    double turretAppliedVx = yawOmega * shooterToInitial[Y];
    double turretAppliedVy = yawOmega * shooterToInitial[X];

    return new double[] {
      fieldRelative[X] + shooterVelocity[X] + turretAppliedVx,
      fieldRelative[Y] + shooterVelocity[Y] + turretAppliedVy,
      fieldRelative[Z] + shooterVelocity[Z]
    };
  }

  protected static double[] initialRotation(double yaw, double heading) {
    return new double[] {0, 0, yaw + heading};
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
    if (speed < EPS) return new double[3];

    double yaw = Math.atan2(shotVelocity[Y], shotVelocity[X]);
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
    protected final double[] targetPose = fromTranslation(Hub.TOP_CENTER_POINT);

    protected double scoreRadius = Hub.INNER_WIDTH + FUEL_RADIUS;
    protected double scoreRadiusSq = scoreRadius * scoreRadius;

    protected boolean inScoringPlane, inScoringRadius;
    private double prevX, prevY, prevZ;

    protected final double[] drag = new double[3];
    protected final double[] lift = new double[3];

    protected Fuel withScoringParameters(double[] goal, double tolerance, double depth) {
      System.arraycopy(goal, 0, targetPose, 0, 3);

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
      return drag;
    }

    @Override
    protected double[] lift() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
      double scale = LIFT_CONSTANT * omega;
      lift[X] = scale * -vx;
      lift[Y] = 0;
      lift[Z] = scale * vz;
      return lift;
    }

    @Override
    protected double torque() {
      return 0;
    }

    @Override
    protected void step() {
      prevX = x;
      prevY = y;
      prevZ = z;

      super.step();

      checkScoringCrossing();
    }

    @Override
    protected boolean willScore() {
      return inScoringPlane && inScoringRadius;
    }

    @Override
    protected boolean willMiss() {
      return (inScoringPlane && !inScoringRadius) || (z < FUEL_RADIUS && vz < 0);
    }

    private void checkScoringCrossing() {
      double planeZ = targetPose[Z] + scoreDepth;

      // CHECK IF PLANE WAS CROSSED
      boolean crossedPlane = prevZ > planeZ && z <= planeZ && vz < 0;

      if (!crossedPlane) {
        inScoringPlane = false;
        return;
      }

      // INTERPOLATION
      double t = (planeZ - prevZ) / (z - prevZ);

      double intersectX = prevX + t * (x - prevX);
      double intersectY = prevY + t * (y - prevY);

      double dx = intersectX - targetPose[X];
      double dy = intersectY - targetPose[Y];

      double distSq = dx * dx + dy * dy;

      inScoringPlane = true;
      inScoringRadius = distSq <= scoreRadiusSq;
    }

    @Override
    protected void reset() {
      super.reset();

      inScoringPlane = false;
      inScoringRadius = false;
    }
  }
}
