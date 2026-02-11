package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public abstract class Projectile {
  public static final double DEFAULT_RESOLUTION = 80;
  public static final int X = 0, Y = 1, Z = 2;

  protected double resolution;
  protected boolean gravityEnabled, dragEnabled, torqueEnabled, liftEnabled;
  protected double[] translation, velocity, acceleration;
  protected double[] rotation, rotationalVelocity, rotationalAcceleration;

  protected abstract double[] gravity();

  protected abstract double[] drag();

  protected abstract double[] torque();

  protected abstract double[] lift();

  protected abstract boolean checkIfScored();

  protected abstract boolean checkIfMissed();

  public Projectile() {
    translation = new double[3];
    velocity = new double[3];
    acceleration = new double[3];

    rotation = new double[3];
    rotationalVelocity = new double[3];
    rotationalAcceleration = new double[3];

    gravityEnabled = true;
    dragEnabled = true;
    torqueEnabled = true;
    liftEnabled = true;

    resolution = DEFAULT_RESOLUTION;
  }

  public void launch(
      double[] launchTranslation,
      double[] launchVelocity,
      double[] launchRotation,
      double[] launchRotationalVelocity) {
    translation = launchTranslation.clone();
    velocity = launchVelocity.clone();
    acceleration = new double[3];

    rotation = launchRotation.clone();
    rotationalVelocity = launchRotationalVelocity.clone();
    rotationalAcceleration = new double[3];
  }

  public Projectile config(
      double fps, boolean gravity, boolean drag, boolean torque, boolean lift) {
    resolution = fps;
    gravityEnabled = gravity;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

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

    if (gravityEnabled) {
      double[] gravity = gravity();

      acceleration[X] += gravity[X];
      acceleration[Y] += gravity[Y];
      acceleration[Z] += gravity[Z];
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

    rotation[X] += rotationalVelocity[X] / resolution;
    rotation[Y] += rotationalVelocity[Y] / resolution;
    rotation[Z] += rotationalVelocity[Z] / resolution;

    rotationalVelocity[X] += rotationalAcceleration[X] / resolution;
    rotationalVelocity[Y] += rotationalAcceleration[Y] / resolution;
    rotationalVelocity[Z] += rotationalAcceleration[Z] / resolution;

    rotationalAcceleration[X] = 0;
    rotationalAcceleration[Y] = 0;
    rotationalAcceleration[Z] = 0;

    if (torqueEnabled) {
      double[] torque = torque();

      rotationalAcceleration[X] = torque[X];
      rotationalAcceleration[Y] = torque[Y];
      rotationalAcceleration[Z] = torque[Z];
    }
  }

  public Pose3d pose() {
    return new Pose3d(
        new Translation3d(translation[X], translation[Y], translation[Z]),
        new Rotation3d(rotation[X], rotation[Y], rotation[Z]));
  }
}
