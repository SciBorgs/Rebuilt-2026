package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.hood.HoodConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HoodVisualizer {
  private final Mechanism2d mech;
  private final MechanismLigament2d hood;
  private final String name;

  /** Offset applied to the AdvantageScope model in order to correct visualizer. */
  private static final Distance ADVANTAGESCOPE_RADIUS_OFFSET = HOOD_RADIUS.minus(Meters.of(0.05));

  private static final Translation3d SHOOTER_TO_FLYWHEEL = new Translation3d(0.105803, 0, 0.061220);

  /**
   * thr constructor for hood visualizer
   *
   * @param name name of the visualizer
   * @param hoodColor color of the hood
   * @param fuelTrajColor color of the line depicting the fuel trajectory
   */
  public HoodVisualizer(String name, Color8Bit hoodColor, Color8Bit fuelTrajColor) {
    this.name = name;
    mech = new Mechanism2d(100, 100);
    MechanismRoot2d chassis = mech.getRoot("Chassis", 50, 10);
    hood =
        chassis.append(
            new MechanismLigament2d(
                "hood", HOOD_RADIUS.in(Inches) * 5, STARTING_ANGLE.in(Degrees), 3, hoodColor));
    hood.append(new MechanismLigament2d("Fuel Trajectory", 25, 90, 2, fuelTrajColor));
  }

  /**
   * sets the visualizer to a certain angle
   *
   * @param angleDegrees the angle to set the visualizer to in degrees
   */
  public void setAngle(double angleDegrees) {
    hood.setAngle(angleDegrees);
    SmartDashboard.putData("/Robot/hood/" + name, mech);
  }

  /**
   * A utility method to accurately display the hood in AdvantageScope.
   *
   * @param hoodAngle the angle of the hood (from angle() method)
   * @param turretAngle the angle of the turret (from position() method)
   * @return a transform from the center of the robot to the pose of the hood
   */
  public static Transform3d transformFromOrigin(double hoodAngle, double turretAngle) {
    double offset = ADVANTAGESCOPE_RADIUS_OFFSET.in(Meters);
    double angle = hoodAngle - MIN_ANGLE.in(Radians);

    double shooterRelativeDisplacement = -offset * Math.cos(angle) + SHOOTER_TO_FLYWHEEL.getX();

    double x = shooterRelativeDisplacement * Math.cos(turretAngle);
    double y = shooterRelativeDisplacement * Math.sin(turretAngle);

    double z = offset * Math.sin(angle) + SHOOTER_TO_FLYWHEEL.getZ();

    return new Transform3d(
        CENTER_TO_SHOOTER.getX() + x,
        CENTER_TO_SHOOTER.getY() + y,
        CENTER_TO_SHOOTER.getZ() + z,
        new Rotation3d(0, angle, turretAngle));
  }
}
