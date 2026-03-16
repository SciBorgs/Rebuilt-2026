package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.EPS;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.units.measure.Distance;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.hood.HoodConstants;

public class ShootingConstants {
  protected static final int SPEED = 0;
  protected static final int PITCH = 1;
  protected static final int YAW = 2;
  protected static final int ERROR = 3;

  protected static final double FUEL_MASS = 0.225;
  protected static final double FUEL_RADIUS = 0.075;

  protected static final Distance SHOOTER_TO_FLYWHEEL = Meters.of(0.105803);
  protected static final Distance FLYWHEEL_LIFT = Meters.of(0.061220);

  protected static final double MAX_AIR_TIME = 10;
  protected static final int TRAJECTORY_RESOLUTION = 200;

  protected static final boolean DRAG_ENABLED = true;
  protected static final boolean LIFT_ENABLED = false;

  protected static final double CLEARANCE = 0.13;
  protected static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

  protected static final double SCORE_DEPTH = 0;
  protected static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

  protected static final int MAX_LOOKUP_TABLE_SIZE = 5000;

  // OPTIMIZATION CONSTANTS
  protected static final double SPEED_KP = 0.5;
  protected static final double SPEED_KD = 0.05;

  protected static final int MAX_OPTIMIZER_ITERATIONS = 300;
  protected static final double OPTIMIZATION_THRESHOLD = 0.01;

  protected static final double MAX_SPEED = 20;
  protected static final double MIN_SPEED = EPS;

  /** The resolution of the lookup table, in entries per meter. */
  protected static final double DISTANCE_RESOLUTION = 100;

  protected static final double MIN_DISTANCE = Hub.WIDTH / 2;
  protected static final double MAX_DISTANCE = 20;

  /** The resolution of the pitches in the lookup table, in samples per 2pi radians. */
  protected static final double PITCH_RESOLUTION = 512;

  protected static final double MIN_PITCH = Math.PI / 2 - HoodConstants.MAX_ANGLE.in(Radians);
  protected static final double MAX_PITCH = Math.PI / 2 - HoodConstants.MIN_ANGLE.in(Radians);

  /** The distance above the Hub from which the FUEL's fate is decided (score / miss). */
  protected static final double SCORE_WINDOW = FUEL_RADIUS / 2;

  /** The target translation for the FUEL to hit. */
  protected static final double[] GOAL = fromTranslation(Hub.TOP_CENTER_POINT);

  /** The path to the lookup table (within the resources folder). */
  protected static final String TABLE_PATH = "shooting/ParameterLookUp";

  /** LookUp Table entries with errors greater than this are removed from the table. */
  protected static final double MAX_ERROR = 1.5;

  /** The translation from the center of the robot the center of the turret */
  public static final double[] ROBOT_TO_SHOOTER = {
    CENTER_TO_SHOOTER.getX(), CENTER_TO_SHOOTER.getY(), CENTER_TO_SHOOTER.getZ()
  };

  /** Multiplied by velocity squared to compute drag acceleration. */
  protected static final double DRAG_CONSTANT =
      0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS / FUEL_MASS;

  /** Multiplied by spin and velocity to produce magnus acceleration. */
  protected static final double LIFT_CONSTANT =
      4.0 / 3.0 * Math.PI * FUEL_RADIUS * FUEL_RADIUS * FUEL_RADIUS * AIR_DENSITY / FUEL_MASS;
}
