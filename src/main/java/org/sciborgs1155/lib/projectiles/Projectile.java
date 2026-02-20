package org.sciborgs1155.lib.projectiles;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** A class that models the physics of a projectile. */
@SuppressWarnings("PMD.OneDeclarationPerLine")
public abstract class Projectile {
  /** The amount of simulation steps per second. */
  public static final double RESOLUTION = 80;

  /** The default period of each simulation step in seconds. */
  public static final double SIMULATION_PERIOD = 1 / RESOLUTION;

  /** The default period of each NetworkTables update in seconds. */
  public static final double LOGGING_PERIOD = PERIOD.in(Seconds);

  /** The X, Y, Z components of a vector are the first, second, and third elements of the array. */
  protected static final int X = 0, Y = 1, Z = 2;

  /** The ANGLE and AXIS components of a rotation vector are placed in that order in the array. */
  protected static final int ANGLE = 0, AXIS_X = 1, AXIS_Y = 2, AXIS_Z = 3;

  protected double resolution;
  protected boolean weightEnabled, dragEnabled, torqueEnabled, liftEnabled;
  protected double[] translation, velocity, acceleration, rotation;
  protected double rotationalVelocity, rotationalAcceleration;

  /**
   * Returns the weight acceleration applied to the projectile based on its current trajectory.
   *
   * @return the weight acceleration applied to the projectile
   */
  protected abstract double[] weight();

  /**
   * Returns the drag acceleration applied to the projectile based on its current trajectory.
   *
   * @return the drag acceleration applied to the projectile
   */
  protected abstract double[] drag();

  /**
   * Returns the torque acceleration applied to the projectile based on its current trajectory.
   *
   * @return the torque acceleration applied to the projectile
   */
  protected abstract double torque();

  /**
   * Returns the lift acceleration applied to the projectile based on its current trajectory.
   *
   * @return the lift acceleration applied to the projectile
   */
  protected abstract double[] lift();

  /**
   * Returns whether the projectile will score based on its current trajectory.
   *
   * @return true if the projectile will score, false otherwise
   */
  protected abstract boolean willScore();

  /**
   * Returns whether the projectile will miss based on its current trajectory.
   *
   * @return true if the projectile will miss, false otherwise
   */
  protected abstract boolean willMiss();

  /** A class that models the physics of a projectile. */
  public Projectile() {
    translation = new double[3];
    velocity = new double[3];
    acceleration = new double[3];

    rotation = new double[4];
    rotationalVelocity = 0;
    rotationalAcceleration = 0;

    weightEnabled = true;
    dragEnabled = true;
    torqueEnabled = true;
    liftEnabled = true;

    resolution = RESOLUTION;
  }

  /**
   * Launches the projectile with the given parameters.
   *
   * @param launchTranslation the initial translation of the projectile at launch time
   * @param launchVelocity the initial velocity of the projectile at launch time
   * @param launchRotation the initial rotation of the projectile at launch time
   * @param launchRotationalVelocity the initial rotational velocity of the projectile at launch
   *     time
   */
  public void launch(
      double[] launchTranslation,
      double[] launchVelocity,
      double[] launchRotation,
      double launchRotationalVelocity) {
    translation = launchTranslation.clone();
    velocity = launchVelocity.clone();
    acceleration = new double[3];

    rotation = launchRotation.clone();
    rotationalVelocity = launchRotationalVelocity;
    rotationalAcceleration = 0;
  }

  /**
   * Configures the projectile's physics settings.
   *
   * @param fps the resolution of the projectile's simulation, in steps per second
   * @param weight Whether to apply weight to the projectile.
   * @param drag Whether to apply drag to the projectile.
   * @param torque Whether to apply torque to the projectile.
   * @param lift Whether to apply lift to the projectile.
   * @return this projectile instance for chaining
   */
  public Projectile config(double fps, boolean weight, boolean drag, boolean torque, boolean lift) {
    resolution = fps;
    weightEnabled = weight;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

  /** Increments the projectile's state by one simulation step. */
  public void periodic() {
    translation[X] += velocity[X] / resolution;
    translation[Y] += velocity[Y] / resolution;
    translation[Z] += velocity[Z] / resolution;

    velocity[X] += acceleration[X] / resolution;
    velocity[Y] += acceleration[Y] / resolution;
    velocity[Z] += acceleration[Z] / resolution;

    acceleration[X] = 0;
    acceleration[Y] = 0;
    acceleration[Z] = 0;

    if (weightEnabled) {
      double[] weight = weight();

      acceleration[X] += weight[X];
      acceleration[Y] += weight[Y];
      acceleration[Z] += weight[Z];
    }

    if (dragEnabled) {
      double[] drag = drag();

      acceleration[X] += drag[X];
      acceleration[Y] += drag[Y];
      acceleration[Z] += drag[Z];
    }

    if (liftEnabled) {
      double[] lift = lift();

      acceleration[X] += lift[X];
      acceleration[Y] += lift[Y];
      acceleration[Z] += lift[Z];
    }

    rotation[ANGLE] += rotationalVelocity / resolution;
    rotationalVelocity += rotationalAcceleration / resolution;
    rotationalAcceleration = 0;

    if (torqueEnabled) {
      rotationalAcceleration = torque();
    }
  }

  /**
   * Returns the projectile's current pose.
   *
   * @return the projectile's current pose
   */
  public Pose3d pose() {
    return new Pose3d(
        translation[X],
        translation[Y],
        translation[Z],
        new Rotation3d(
            VecBuilder.fill(rotation[AXIS_X], rotation[AXIS_Y], rotation[AXIS_Z]),
            rotation[ANGLE]));
  }

  protected static double[] fromTranslation(Translation3d translation) {
    return new double[] {translation.getX(), translation.getY(), translation.getZ()};
  }

  protected static double[] toDirectionVector(double pitch, double yaw) {
    return new double[] {
      Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)
    };
  }

  protected static double[] rotateAroundZ(double[] vector, double angle) {
    return new double[] {
      vector[X] * Math.cos(angle) - vector[Y] * Math.sin(angle),
      vector[X] * Math.sin(angle) + vector[Y] * Math.cos(angle),
      vector[Z]
    };
  }

  protected static double[] add3(double[] vector1, double[] vector2) {
    return new double[] {vector1[X] + vector2[X], vector1[Y] + vector2[Y], vector1[Z] + vector2[Z]};
  }

  protected static double[] scale3(double[] vector, double scalar) {
    return new double[] {vector[X] * scalar, vector[Y] * scalar, vector[Z] * scalar};
  }

  protected static double[] scale4(double[] vector, double scalar) {
    return new double[] {
      vector[ANGLE] * scalar,
      vector[AXIS_X] * scalar,
      vector[AXIS_Y] * scalar,
      vector[AXIS_Z] * scalar
    };
  }

  protected static double norm3(double[] vector) {
    return Math.sqrt(vector[X] * vector[X] + vector[Y] * vector[Y] + vector[Z] * vector[Z]);
  }
}
