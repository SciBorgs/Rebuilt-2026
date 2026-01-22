package org.sciborgs1155.robot.commands;

/** Command factory for scoring Fuel into the Hub. All units are SI. */
public class Scoring {
  // /** Controls the position of the robot. */
  // // private final Drive drive;

  // /** Controls the speed at which Fuel is launched. */
  // // private final Shooter shooter;

  // /** Controls the yaw component of the Fuel's launch angle. */
  // // private final Turret turret;

  // /** Controls the pitch component of the Fuel's launch angle. */
  // // private final Hood hood;

  // /**
  //  * Command factory for scoring Fuel into the Hub.
  //  *
  //  * @param drive The {@code Drive} subsystem (controls launch position).
  //  * @param shooter The {@code Shooter} subsystem (controls launch speed).
  //  * @param turret The {@code Turret} subsystem (controls launch yaw).
  //  * @param hood The {@code Hood} subsystem (controls launch pitch).
  //  */
  // // public Scoring(Drive drive, Shooter shooter, Turret turret, Hood hood) {
  // //   this.drive = drive;
  // //   this.shooter = shooter;
  // //   this.turret = turret;
  // //   this.hood = hood;
  // // }

  // /**
  //  * Continuously command all scoring mechanisms to the appropriate configuration for scoring
  // Fuel
  //  * into the Hub.
  //  *
  //  * @return A command that continuously updates the {@code Shooter} power, {@code Turret} yaw,
  // and
  //  *     {@code Hood} pitch based on the {@code Drive}'s position such that Fuel will always be
  //  *     scored into the Hub.
  //  */
  // public Command run() {
  //   // TODO: Implement.
  //   return Commands.idle();
  // }

  // /**
  //  * Whether it is possible to score Fuel into the Hub from the current robot position.
  //  *
  //  * @return True if scoring is possible. False if scoring is not possible.
  //  */
  // @Logged(name = "IN RANGE")
  // public boolean inRange() {
  //   // TODO: Implement.
  //   return false;
  // }

  // /**
  //  * Calculates the speed of the {@code Shooter} roller based on a target launch speed for the
  // Fuel.
  //  *
  //  * @param fuelSpeed The launch speed of the Fuel (after leaving the {@code Shooter}).
  //  * @return The speed of the rollers on the {@code Shooter}.
  //  */
  // private double getShooterVelocity(double fuelSpeed) {
  //   // TODO: Implement.
  //   return 0;
  // }

  // /**
  //  * Calculates the pitch of the {@code Hood} based on a target launch pitch for the Fuel.
  //  *
  //  * @param fuelLaunchPitch The launch pitch of the Fuel (after leaving the {@code Shooter}).
  //  * @return The angle of the {@code Hood}.
  //  */
  // private double getHoodAngle(double fuelLaunchPitch) {
  //   // TODO: Implement.
  //   return 0;
  // }

  // /**
  //  * Calculates the yaw of the {@code Turret} based on a target launch yaw for the Fuel.
  //  *
  //  * @param fuelLaunchYaw The launch yaw of the Fuel (after leaving the {@code Shooter}).
  //  * @return The angle of the {@code Turret}.
  //  */
  // private double getTurretAngle(double fuelLaunchYaw) {
  //   // TODO: Implement.
  //   return 0;
  // }
}
