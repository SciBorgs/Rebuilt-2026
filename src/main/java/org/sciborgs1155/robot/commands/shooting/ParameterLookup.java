package org.sciborgs1155.robot.commands.shooting;

/**
 * A class used to analyze and compile shot data into a polynomial regression model that can be used
 * to determine the direct launch parameters for a given distance.
 */
public final class ParameterLookup {

  // PREVENTS INSTANTIATION
  private ParameterLookup() {}

  /**
   * The speed of the rollers on the shooter flywheel required to produce the given launch speed
   * estimated using the model (radians per second).
   *
   * @param speed the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double rollerSpeed(double speed) {
    return 0;
  }

  /**
   * The launch speed of the FUEL produced by the shooter rollers estimated using the model (meters
   * per second).
   *
   * @param rollerSpeed the speed of the rollers on the shooter flywheel, in radians per second
   */
  public static double speedFromRollers(double rollerSpeed) {
    return 0;
  }

  /**
   * The launch speed of the FUEL from the given distance estimated using the model (meters per
   * second).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double speed(double distance) {
    return 0;
  }

  /**
   * The launch pitch of the FUEL from the given distance estimated using the model (radians).
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double pitch(double distance) {
    return 0;
  }

  /**
   * The horizontal error of a launch from the given distance using the model (meters). Simulation
   * ends when the FUEL either hits the ground or hits the horizontal plane formed by the rim of the
   * hub while going downwards.
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double estimatedError(double distance) {
    return 0;
  }

  /**
   * The time-of-flight of a launch from the given distance using the model (seconds). Simulation
   * ends when the FUEL either hits the ground or hits the horizontal plane formed by the rim of the
   * hub while going downwards.
   *
   * @param distance the planar distance from the HUB to the shooter's origin, in meters
   */
  public static double timeOfFlight(double distance) {
    return 0;
  }
}
