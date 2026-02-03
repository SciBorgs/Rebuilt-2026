package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.lib.projectiles.FuelVisualizer.*;
import static org.sciborgs1155.robot.FieldConstants.BLUE_HUB;
import static org.sciborgs1155.robot.FieldConstants.HUB_DIAMETER;
import static org.sciborgs1155.robot.FieldConstants.HUB_HEIGHT;
import static org.sciborgs1155.robot.FieldConstants.RED_HUB;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/** The Fuel projectile from REBUILT® 2026. */
public class Fuel extends Projectile {
  @Override
  protected Vector<N3> acceleration() {
    // GRAVITY CALCULATIONS (METERS / FRAME^2)
    Vector<N3> gravity = VecBuilder.fill(0, 0, GRAVITY).times(Math.pow(FRAME_LENGTH, 2));

    // DRAG CALCULATIONS (METERS / FRAME^2)
    double speedSquared = Math.pow(velocity.norm(), 2); // MAGNITUDE OF VELOCITY^2
    double dragCoefficient = 0.47; // SPECIFIC TO SPHERICAL OBJECTS
    double referenceArea = Math.PI * Math.pow(FUEL_RADIUS, 2); // CROSS SECTION
    double dragFactor = 0.5 * AIR_DENSITY * speedSquared * dragCoefficient * referenceArea;
    Vector<N3> drag = velocity.unit().times(dragFactor).div(FUEL_MASS).times(-1);

    return gravity.plus(drag);
  }

  // TODO: Implement.
  @Override
  protected Vector<N3> rotationalAcceleration() {
    return VecBuilder.fill(0, 0, 0);
  }

  @Override
  protected boolean isScoring() {
    Translation2d fuelTranslation = new Translation2d(translation.get(0), translation.get(1));

    double distanceFromBlueHub = fuelTranslation.getDistance(BLUE_HUB);
    double distanceFromRedHub = fuelTranslation.getDistance(RED_HUB);

    // FUEL MUST BE DIRECTLY ABOVE THE HUB
    double verticalDisplacement = HUB_HEIGHT.in(Meters) - translation.get(2);

    // FUEL MUST BE CLOSE TO THE HUB ON THE 2D PLANE
    double scoreRadius = FUEL_RADIUS + HUB_DIAMETER.in(Meters) / 2;

    return (verticalDisplacement < 0)
        && (verticalDisplacement > -FUEL_RADIUS)
        && (distanceFromBlueHub < scoreRadius || distanceFromRedHub < scoreRadius)
        // FUEL MUST HAVE A DOWNWARD VELOCITY
        && velocity.get(2) < 0;
  }

  @Override
  protected boolean isMissing() {
    return translation.get(2) < FUEL_RADIUS;
  }
}
