package org.sciborgs1155.lib.projectiles;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public abstract class Projectile {
  public static final int RESOLUTION = 50;
  public static final int X = 0, Y = 1, Z = 2;
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

  public Projectile config(boolean gravity, boolean drag, boolean torque, boolean lift) {
    gravityEnabled = gravity;
    dragEnabled = drag;
    torqueEnabled = torque;
    liftEnabled = lift;

    return this;
  }

  public void periodic() {
    translation[X] += velocity[X] / RESOLUTION;
    translation[Y] += velocity[Y] / RESOLUTION;
    translation[Z] += velocity[Z] / RESOLUTION;

    velocity[X] += acceleration[X] / RESOLUTION;
    velocity[Y] += acceleration[Y] / RESOLUTION;
    velocity[Z] += acceleration[Z] / RESOLUTION;

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

    rotation[X] += rotationalVelocity[X] / RESOLUTION;
    rotation[Y] += rotationalVelocity[Y] / RESOLUTION;
    rotation[Z] += rotationalVelocity[Z] / RESOLUTION;

    rotationalVelocity[X] += rotationalAcceleration[X] / RESOLUTION;
    rotationalVelocity[Y] += rotationalAcceleration[Y] / RESOLUTION;
    rotationalVelocity[Z] += rotationalAcceleration[Z] / RESOLUTION;

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
