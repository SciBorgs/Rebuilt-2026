package org.sciborgs1155.lib.projectiles;

import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.FRAME_LENGTH;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.PITCH;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.ROLL;
import static org.sciborgs1155.lib.projectiles.ProjectileVisualizer.YAW;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/** An interface for a projectile to be visualized by the {@code ProjectileVisualizer}. */
public abstract class Projectile {
  /**
   * @see {@link #scored() The Usage. }
   */
  private boolean scored;

  /**
   * @see {@link #missed() The Usage. }
   */
  private boolean missed;

  /**
   * @see {@link #isBeingLaunched() The Usage. }
   */
  private boolean isBeingLaunched;

  /**
   * @see {@link #hasBeenLaunched() The Usage. }
   */
  private boolean hasBeenLaunched;

  /**
   * @see {@link #scores() The Usage. }
   */
  private int scores;

  /**
   * @see {@link #misses() The Usage. }
   */
  private int misses;

  /**
   * @see {@link #translation() The Usage. }
   */
  private Vector<N3> translation = VecBuilder.fill(0, 0, 0);

  /**
   * @see {@link #velocity() The Usage. }
   */
  private Vector<N3> velocity = VecBuilder.fill(0, 0, 0);

  /**
   * @see {@link #rotation() The Usage. }
   */
  private Vector<N3> rotation = VecBuilder.fill(0, 0, 0);

  /**
   * @see {@link #rotationalVelocity() The Usage. }
   */
  private Vector<N3> rotationalVelocity = VecBuilder.fill(0, 0, 0);

  /**
   * The current acceleration of the projectile.
   *
   * @return The field-relative acceleration (METERS / FRAME^2).
   */
  public abstract Vector<N3> acceleration();

  /**
   * The current rotational acceleration of the projectile.
   *
   * @return The acceleration (RADIANS / FRAME^2).
   */
  public abstract Vector<N3> rotationalAcceleration();

  /**
   * Displays the current pose of the projectile.
   *
   * @return The current field-relative pose of the projectile (METERS).
   */
  public Pose3d pose() {
    return new Pose3d(
        new Translation3d(translation),
        new Rotation3d(rotation.get(ROLL), rotation.get(PITCH), rotation.get(YAW)));
  }

  /**
   * Starts frame generation.
   *
   * @param launchTranslation The field-relative translation of the projectile at launch (METERS).
   * @param launchVelocity The field-relative velocity of the projectile at launch (METERS PER
   *     SECOND).
   * @param launchRotation The projectile-relative rotation of the projectile (RADIANS).
   * @param launchRotationalVelocity The projectile-relative rotational velocity of the projectile
   *     (RADIANS PER SECOND).
   */
  public void launch(
      Vector<N3> launchTranslation,
      Vector<N3> launchVelocity,
      Vector<N3> launchRotation,
      Vector<N3> launchRotationalVelocity) {
    if (isBeingLaunched) return;

    translation = launchTranslation;
    velocity = launchVelocity.times(FRAME_LENGTH);

    rotation = launchRotation;
    rotationalVelocity = launchRotationalVelocity.times(FRAME_LENGTH);

    // ALLOWS FRAMES TO BE RENDERED
    isBeingLaunched = true;
  }

  /** Calculates and displays the next frame in the projectile animation. */
  public void nextFrame() {
    if (!isBeingLaunched) return;

    velocity.setColumn(0, velocity.plus(acceleration()));
    translation.setColumn(0, translation.plus(velocity));

    rotationalVelocity.setColumn(0, rotationalVelocity.plus(rotationalAcceleration()));
    rotation.setColumn(0, rotation.plus(rotationalVelocity));

    if (isMissing() || isScoring()) end();
  }

  /** Disables frame generation. */
  public void end() {
    if (isMissing()) {
      scored = false;
      missed = true;
      misses++;
    }

    if (isScoring()) {
      scored = true;
      missed = false;
      scores++;
    }

    // PREVENTS DISPLAY
    translation = VecBuilder.fill(0, 0, 0);
    velocity = VecBuilder.fill(0, 0, 0);

    rotation = VecBuilder.fill(0, 0, 0);
    rotationalVelocity = VecBuilder.fill(0, 0, 0);

    // PREVENTS FRAMES FROM BEING RENDERED
    isBeingLaunched = false;
    hasBeenLaunched = true;
  }

  /**
   * Determines if the projectile is destined to score.
   *
   * @return If the projectile is destined to score, returns True. If not, or it is not
   *     determinable, return False.
   */
  public abstract boolean isScoring();

  /**
   * True if the projectile scored it's latest shot.
   *
   * @return If the projectile scored it's latest shot, returns True.
   */
  public boolean scored() {
    return scored;
  }

  /**
   * The total amount of times this projectile has been scored.
   *
   * @return The total amount of times this projectile has been scored.
   */
  public int scores() {
    return scores;
  }

  /**
   * Determines if the projectile is destined to miss.
   *
   * @return If the projectile is destined to miss, returns True. If not, or it is not determinable,
   *     return False.
   */
  public abstract boolean isMissing();

  /**
   * True if the projectile missed it's latest shot.
   *
   * @return If the projectile missed it's latest shot, returns True.
   */
  public boolean missed() {
    return missed;
  }

  /**
   * The total amount of times this projectile has missed.
   *
   * @return The total amount of times this projectile has missed.
   */
  public int misses() {
    return misses;
  }

  /**
   * True if the projectile has been launched at least once.
   *
   * @return If the projectile has been launched at least once, returns True.
   */
  public boolean hasBeenLaunched() {
    return hasBeenLaunched;
  }

  /**
   * True if the projectile is not generating new frames.
   *
   * @return If the projectile is not generating new frames, returns True.
   */
  public boolean isBeingLaunched() {
    return isBeingLaunched;
  }

  /**
   * The field-relative translation of the projectile.
   *
   * @return The field-relative translation of the projectile (METERS).
   */
  public Vector<N3> translation() {
    return translation;
  }

  /**
   * The field-relative velocity of the projectile.
   *
   * @return The field-relative velocity of the projectile (METERS / FRAME).
   */
  public Vector<N3> velocity() {
    return velocity;
  }

  /**
   * The field-relative rotation of the projectile.
   *
   * @return The field-relative rotation of the projectile (RADIANS).
   */
  public Vector<N3> rotation() {
    return rotation;
  }

  /**
   * The field-relative rotational velocity of the projectile.
   *
   * @return The field-relative rotational velocity of the projectile (RADIANS / FRAME).
   */
  public Vector<N3> rotationalVelocity() {
    return rotationalVelocity;
  }
}
