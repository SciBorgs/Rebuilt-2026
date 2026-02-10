package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.AIR_DENSITY;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.AIR_VISCOSITY;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.FRAME_LENGTH;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.GRAVITY;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.X;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.Y;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.Z;
import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;
import static org.sciborgs1155.robot.FieldConstants.FUEL_MASS;
import static org.sciborgs1155.robot.FieldConstants.FUEL_RADIUS;
import static org.sciborgs1155.robot.FieldConstants.HUB_DIAMETER;
import static org.sciborgs1155.robot.FieldConstants.HUB_HEIGHT;
import static org.sciborgs1155.robot.FieldConstants.RED_HUB;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/** The Fuel projectile from REBUILT® 2026. */
public class Fuel extends Projectile {
  /**
   * @see {@link #acceleration() The Usage.}
   */
  private static final double DRAG_CONSTANT =
      -0.5 * AIR_DENSITY * 0.47 * Math.PI * Math.pow(FUEL_RADIUS, 2) / FUEL_MASS;

  /**
   * @see {@link #acceleration() The Usage.}
   */
  private static final double GRAVITY_CONSTANT = GRAVITY * Math.pow(FRAME_LENGTH, 2);

  /**
   * @see {@link #rotationalAcceleration() The Usage.}
   */
  private static final double VISCOUS_DRAG_CONSTANT =
      -8 * Math.PI * Math.pow(FUEL_RADIUS, 3) * AIR_VISCOSITY / FUEL_MASS;

  @Override
  public Vector<N3> acceleration() {
    // GRAVITY CALCULATIONS (METERS / FRAME^2)
    // SOURCE: https://spaceplace.nasa.gov/what-is-gravity/en/
    Vector<N3> gravity = VecBuilder.fill(0, 0, GRAVITY_CONSTANT);

    // DRAG CALCULATIONS (METERS / FRAME^2)
    // SOURCE: https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
    Vector<N3> drag = velocity().unit().times(Math.pow(velocity().norm(), 2) * DRAG_CONSTANT);

    // MAGNUS LIFT CALCULATIONS (METERS / FRAME^2)
    // SOURCE:
    // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
    double airSpeed = velocity().norm(); // RELATIVE TO FUEL
    double angularSpeed = rotationalVelocity().norm();
    Vector<N3> direction = new Rotation3d(velocity(), Math.PI / 2).toVector();
    double magnusNorm =
        airSpeed * 4 * Math.pow(Math.PI, 2) * Math.pow(FUEL_RADIUS, 3) * angularSpeed * AIR_DENSITY;
    Vector<N3> magnusLift = direction.unit().times(magnusNorm).div(FUEL_MASS);

    return gravity.plus(drag);
  }

  @Override
  public Vector<N3> rotationalAcceleration() {
    // VISCOUS TORQUE CALCULATIONS (METERS / FRAME^2)
    // SOURCE:
    // https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_1998_Grugel.pdf
    return rotationalVelocity().times(VISCOUS_DRAG_CONSTANT);
  }

  @Override
  public boolean isScoring() {
    Translation2d fuelTranslation = new Translation2d(translation().get(X), translation().get(Y));

    double distanceFromBlueHub = fuelTranslation.getDistance(BLUE_HUB.toTranslation2d());
    double distanceFromRedHub = fuelTranslation.getDistance(RED_HUB);

    // FUEL MUST BE DIRECTLY ABOVE THE HUB
    double verticalDisplacement = HUB_HEIGHT.in(Meters) - translation().get(Z);

    // FUEL MUST BE CLOSE TO THE HUB ON THE 2D PLANE
    double scoreRadius = FUEL_RADIUS + HUB_DIAMETER.in(Meters) / 2;

    return (verticalDisplacement < 0)
        && (verticalDisplacement > -FUEL_RADIUS)
        && (distanceFromBlueHub < scoreRadius || distanceFromRedHub < scoreRadius)
        // FUEL MUST HAVE A DOWNWARD VELOCITY
        && velocity().get(Z) < 0;
  }

  @Override
  public boolean isMissing() {
    return translation().get(2) < FUEL_RADIUS;
  }
}
