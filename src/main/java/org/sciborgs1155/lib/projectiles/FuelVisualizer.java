package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.FieldConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

/** Simulates the behavior of multiple Fuel projectiles using {@code FuelSim}. */
public class FuelVisualizer extends ProjectileVisualizer {
  /** The mass of the Fuel (KILOGRAMS). */
  protected static final double FUEL_MASS = 0.225;

  /** The radius of the Fuel (METERS). */
  protected static final double FUEL_RADIUS = 0.075;

  /** The radius of the shooter wheel (METERS). */
  protected static final double SHOOTER_WHEEL_RADIUS = 0.1016;

  /** The mass of the shooter wheel (KILOGRAMS). */
  protected static final double SHOOTER_WHEEL_MASS = 0.27215542;

  /** The robot-relative translation of the shooter (METERS). */
  protected static final Translation3d ROBOT_TO_SHOOTER =
      new Translation3d(0.5, 0.5, 0.5); // TODO: UPDATE.

  /**
   * The distance between the shooter origin and the point where the Fuel is in free-fall. (METERS).
   */
  protected static final Distance SHOOTER_LENGTH = Meters.of(0.1); // TODO: UPDATE.

  /** A supplier for the angular velocity of the wheel in the {@code Shooter}. */
  protected final Supplier<AngularVelocity> wheelVelocity;

  /** A supplier for the angle of the {@code Hood}. */
  protected final Supplier<Angle> hoodAngle;

  /** A supplier for the angle of the {@code Turret}. */
  protected final Supplier<Angle> turretAngle;

  /**
   * Simulates the behavior of multiple {@code Fuel} projectiles. Parameters used to calculate Fuel
   * trajectory after launch.
   *
   * @param wheelVelocitySupplier A supplier for the angular velocity of the wheel in the {@code
   *     Shooter}.
   * @param turretAngleSupplier A supplier for the angle of the {@code Turret}.
   * @param hoodAngleSupplier A supplier for the angle of the {@code Hood}.
   * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
   * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
   */
  public FuelVisualizer(
      Supplier<AngularVelocity> wheelVelocitySupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier,
      Supplier<Pose3d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier) {
    super(robotPoseSupplier, robotVelocitySupplier);
    wheelVelocity = wheelVelocitySupplier;
    turretAngle = turretAngleSupplier;
    hoodAngle = hoodAngleSupplier;
  }

  @Override
  protected double launchSpeed() {
    // Used "Wheel Velocity with Shape Factors" equation from below source.
    // https://www.chiefdelphi.com/t/new-flywheel-shooter-analysis/439111/4

    double massRatio = FUEL_MASS / SHOOTER_WHEEL_MASS;
    double numerator = SHOOTER_WHEEL_RADIUS * wheelVelocity.get().in(RadiansPerSecond);
    double shapeFactor = 7 / 5;

    return numerator / (2 + shapeFactor * massRatio);
  }

  @Override
  protected Vector<N3> launcherVelocity(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    Vector<N2> rotationalVelocity =
        fromPolarCoords(
            robotVelocity.omegaRadiansPerSecond
                * ROBOT_TO_SHOOTER
                    .toVector()
                    .plus(launchDirection(robotPose).times(SHOOTER_LENGTH.in(Meters)))
                    .norm(),
            robotPose.toPose2d().getRotation().plus(Rotation2d.kCCW_90deg));
    Vector<N2> translationalVelocity =
        VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    Vector<N2> shooter2DVelocity = translationalVelocity.plus(rotationalVelocity);

    return VecBuilder.fill(shooter2DVelocity.get(0), shooter2DVelocity.get(1), 0);
  }

  @Override
  protected Vector<N3> launchDirection(Pose3d robotPose) {
    return fromSphericalCoords(
        1,
        new Rotation3d(Radians.zero(), hoodAngle.get(), turretAngle.get())
            .rotateBy(robotPose.getRotation()));
  }

  @Override
  protected Vector<N3> launchTranslation(Pose3d robotPose) {
    return ROBOT_TO_SHOOTER
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation())
        .toVector()
        .plus(launchDirection(robotPose).times(SHOOTER_LENGTH.in(Meters)));
  }

  @Override
  protected Projectile createProjectile() {
    return new Fuel();
  }

  /** A modded {@code FuelVisualizer} to be compatible with vector input. */
  public class FuelVectorVisualizer extends FuelVisualizer {
    /** A supplier for the launch velocity of the Fuel. */
    protected final Supplier<Vector<N3>> launchVelocityVector;

    /**
     * A modded {@code FuelVisualizer} to be compatible with vector input
     *
     * @param launchVelocityVectorSupplier A supplier for the launch velocity of the Fuel.
     * @param robotPoseSupplier A supplier for the pose of the {@code Drive}.
     * @param robotVelocitySupplier A supplier for the velocity of the {@code Drive}.
     */
    public FuelVectorVisualizer(
        Supplier<Vector<N3>> launchVelocityVectorSupplier,
        Supplier<Pose3d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier) {
      super(
          RadiansPerSecond::zero,
          Radians::zero,
          Radians::zero,
          robotPoseSupplier,
          robotVelocitySupplier);
      launchVelocityVector = launchVelocityVectorSupplier;
    }

    @Override
    protected double launchSpeed() {
      return launchVelocityVector.get().norm();
    }

    @Override
    protected Vector<N3> launchDirection(Pose3d robotPose) {
      return launchVelocityVector.get().unit();
    }

    /**
     * Utility method for calculating velocity vector.
     *
     * @param robotPose The current pose of the robot.
     * @return The field-relative translation of the shooter.
     */
    public static Translation2d shooterTranslation(Pose3d robotPose) {
      return ROBOT_TO_SHOOTER
          .rotateBy(robotPose.getRotation())
          .plus(robotPose.getTranslation())
          .toTranslation2d();
    }

    /**
     * Utility method for calculating velocity vector. <br>
     * </br> NOTE: Does not account for robot rotational velocity.
     *
     * @param robotPose The current pose of the robot.
     * @param robotVelocity The robot-relative velocity of the robot.
     * @return The field-relative translation of the shooter.
     */
    public static Vector<N2> shooterVelocity(ChassisSpeeds robotVelocity) {
      return VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    }
  }
}
