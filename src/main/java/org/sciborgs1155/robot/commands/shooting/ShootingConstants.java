package org.sciborgs1155.robot.commands.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.AIR_DENSITY;
import static org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer.Projectile.fromTranslation;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import org.sciborgs1155.robot.FieldConstants.Hub;
import org.sciborgs1155.robot.hood.HoodConstants;

public class ShootingConstants {
  public static final int SPEED = 0, PITCH = 1, YAW = 2;

  public static final double FUEL_MASS = 0.225;
  public static final double FUEL_RADIUS = 0.075;

  public static final Distance SHOOTER_TO_FLYWHEEL = Meters.of(0.105803);
  public static final Distance FLYWHEEL_LIFT = Meters.of(0.061220);
  public static final Translation3d ROBOT_TO_SHOOTER =
      new Translation3d(0.14006, 0.13983, 0.3286252);

  public static final double SPEED_KP = 0.5;
  public static final double SPEED_KD = 0.05;

  public static final int OPTIMIZATION_RESOLUTION = 300;
  public static final double OPTIMIZATION_THRESHOLD = 0.01;

  public static final double MAX_AIR_TIME = 10;
  public static final int TRAJECTORY_RESOLUTION = 200;
  public static final double MAX_FRAMES = TRAJECTORY_RESOLUTION * MAX_AIR_TIME;

  public static final boolean DRAG_ENABLED = true;
  public static final boolean LIFT_ENABLED = false;

  public static final double CLEARANCE = 0.13;
  public static final double CLEARANCE_CHECK = Hub.INNER_WIDTH / 2;

  public static final double SCORE_DEPTH = 0;
  public static final double SCORE_RADIUS = Hub.INNER_WIDTH / 2;

  public static final double PITCH_PRECISION = Math.PI / 256;

  public static final double MAX_SPEED = 20;
  public static final double MIN_PITCH = Math.PI / 2 - HoodConstants.MAX_ANGLE.in(Radians);
  public static final double MAX_PITCH = Math.PI / 2 - HoodConstants.MIN_ANGLE.in(Radians);

  public static final double SCORE_WINDOW = FUEL_RADIUS / 2;
  public static final double[] GOAL = fromTranslation(Hub.TOP_CENTER_POINT);

  public static final double MIN_DISTANCE = Hub.WIDTH / 2;
  public static final double MAX_DISTANCE = 20;

  public static final double INCREMENT = 0.005;
  public static final String PATH = "shooting/ParameterLookUp";

  public static final double SPEED_DEADBAND = 0.01;
  public static final double ERROR_THRESHOLD = 1.5;

  public static final int MAX_TABLE_SIZE = 5000;
  public static final int ERROR = 3;

  public static final Translation3d HOOD_ORIGIN = new Translation3d(0.14006, 0.13983, 0.4086252);

  /** Multiplied by velocity squared to compute drag acceleration. */
  public static final double DRAG_CONSTANT =
      0.5 * 0.47 * AIR_DENSITY * Math.PI * FUEL_RADIUS * FUEL_RADIUS / FUEL_MASS;

  /** Multiplied by spin and velocity to produce magnus acceleration. */
  public static final double LIFT_CONSTANT =
      4.0 / 3.0 * Math.PI * FUEL_RADIUS * FUEL_RADIUS * FUEL_RADIUS * AIR_DENSITY / FUEL_MASS;
}
