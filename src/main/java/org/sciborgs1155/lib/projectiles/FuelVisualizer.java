package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.Robot.*;
import static org.sciborgs1155.robot.FieldConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;

/** Simulates the behavior of multiple Fuel projectiles using {@code FuelSim}. */
public class FuelVisualizer extends ProjectileVisualizer {
  /**
   * @see {@link #launchVelocity(Pose3d, ChassisSpeeds) The Usage.}
   */
  protected static final double SPEED_CONSTANT =
      SHOOTER_WHEEL_RADIUS / (2 + 7 / 5 * (FUEL_MASS / SHOOTER_WHEEL_MASS));

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

  /**
   * Calculates the launch direction of the projectile.
   *
   * @param robotPose The current pose of the robot.
   * @return The field-relative launch direction of the projectile (UNIT VECTOR).
   */
  protected Vector<N3> launchDirection(Pose3d robotPose) {
    return fromSphericalCoords(
        1,
        new Rotation3d(Radians.zero(), hoodAngle.get(), turretAngle.get())
            .rotateBy(robotPose.getRotation()));
  }

  /**
   * Calculates the robot-relative launch translation of the projectile.
   *
   * @param robotPose The current pose of the robot.
   * @return The robot-relative launch translation of the projectile (METERS).
   */
  protected Vector<N3> robotToFuel(Pose3d robotPose) {
    return ROBOT_TO_SHOOTER
        .toVector()
        .plus(launchDirection(robotPose).times(SHOOTER_LENGTH.in(Meters)));
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
  protected Vector<N3> launchVelocity(Pose3d robotPose, ChassisSpeeds robotVelocity) {
    // SPEED CALCULATIONS
    // Used "Wheel Velocity with Shape Factors" equation from below source.
    // https://www.chiefdelphi.com/t/new-flywheel-shooter-analysis/439111/4

    double launchSpeed = SPEED_CONSTANT * wheelVelocity.get().in(RadiansPerSecond);

    // SHOOTER ROTATIONAL VELOCITY
    Vector<N2> rotationalVelocity =
        fromPolarCoords(
            robotVelocity.omegaRadiansPerSecond * robotToFuel(robotPose).norm(),
            robotPose.toPose2d().getRotation().plus(Rotation2d.kCCW_90deg));

    return launchDirection(robotPose)
        .times(launchSpeed)
        .plus(
            VecBuilder.fill(
                robotVelocity.vxMetersPerSecond + rotationalVelocity.get(X),
                robotVelocity.vyMetersPerSecond + rotationalVelocity.get(Y),
                0)); // THE ROBOT ISN'T LIFTING OFF THE GROUND (FOR NOW)
  }

  @Override
  protected Vector<N2> launchRotation(Pose3d robotPose) {
    return VecBuilder.fill(0, 0);
  }

  @Override
  protected Vector<N2> launchRotationalVelocity(Pose3d robotPose) {
    return VecBuilder.fill(0, 0);
  }

  @Override
  protected Projectile createProjectile() {
    return new Fuel();
  }
}
