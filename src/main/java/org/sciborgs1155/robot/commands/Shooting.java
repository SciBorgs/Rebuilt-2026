package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {
  private final Shooter shooter;
  private final Drive drive;

  public Shooting(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
  }

  /**
   *  More general TODOs
   *  - Replace feeder with / spindexer / hopper / something else
   *  - Make sure the chain of logic still makes sense and that the information applies to 2026 robot
   *  - Make mini-wrappers that are just the names of the functions of each component 
   * --> just to make it more readable --> potentially have a more drafty version of each for looking at issues just at a glance 
   * 
   * Next Steps: 
   *  - Look more into MoSim, AdvantageScope, Visualization, and just talking to Ankit more about those things generally
   *  - look into how to modify or use the lookup table more
   *  - try to make a virtual lookup table to be determeinded or replaced with the more real life one later
   *  - try to work with more air resistance and elasticity stuff // , variablity of speed vs other stuff, boundary and range of shot 
   * (where within the hopper should the fuel be aimed in which types of sitations --> something which has yet to be fully defined)
   * 
   * 
   */

  /**
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity in radians per second to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(AngularVelocity desiredVelocity) {
    return null;
  }

  /**
   * Runs shooter to desired velocity, runs feeder once it reaches its velocity and shootCondition
   * is true.
   *
   * @param desiredVelocity Target velocity for the flywheel.
   * @param shootCondition Condition after which the feeder will run.
   */
  public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
    return Commands.waitUntil(
            () ->
                shooter.atVelocity(desiredVelocity.getAsDouble()) && shootCondition.getAsBoolean())
        .andThen(feeder.eject())
        .deadlineWith(shooter.runShooter(desiredVelocity));
  }

  /**
   * I just want to write down some Pseudocode for now, since it seems like this is probably
   * something that would not get done until a lot later. I will be mostly repurposing and reusing
   * most of these commands
   *
   * <p>--> it would be best if I can softcode this into a sort of utility class --> it would be
   * pretty cool if we could give something like this its own repository, too!
   *
   * <p>shootWhileDriving (InputStream vx, vy) { return shoot( () ->
   * rotationalVelocityFromNoteVelocity(CalculateFuelVelocity()); //I wonder if this is a new
   * calation because the shooters's angular velocity doesn't translate perfectly into note-velocity
   * (translational and rotational) () ->
   * turret.atPosition(yawFromFuelVelocity(calculateFuelVelocity()));
   *
   * <p>) }
   *
   * <p>I think that this structure is one of the most important to implement:
   *
   * <p>public Vector<N3> calculateNoteVelocity(Pose2d robotPose) { ChassisSpeeds speeds =
   * drive.getFieldRelativeChassisSpeeds(); Vector<N3> robotVelocity =
   * VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0); Translation2d
   * difference = translationToSpeaker(robotPose.getTranslation()); double shotVelocity =
   * calculateStationaryVelocity(difference.getNorm()); Rotation3d noteOrientation = new Rotation3d(
   * 0, -calculateStationaryPitch( robotPoseFacingSpeaker(robotPose.getTranslation()), shotVelocity,
   * pivot.position()), difference.getAngle().getRadians()); // rotate unit forward vector by note
   * orientation and scale by our shot velocity Vector<N3> noteVelocity = new Translation3d(1, 0,
   * 0).rotateBy(noteOrientation).toVector().unit().times(shotVelocity);
   *
   * <p>return noteVelocity.minus(robotVelocity); }
   */

  /**
   * Shoots while driving at a manually inputted translational velocity.
   *
   * @param vx The field relative x velocity to drive in.
   * @param vy The field relative y velocity to drive in.
   * @return A command to shote while moving.
   */
  public Command shootWhileDriving(
      InputStream vx, InputStream vy) { // this seems like something that could be mostly kept
    return shoot(
            () -> rotationalVelocityFromNoteVelocity(calculateNoteVelocity()),
            () -> atYaw(yawFromNoteVelocity(calculateNoteVelocity())))
        .deadlineWith(
            drive.drive(
                vx.scale(
                    0.5), // see if we could speed this up in the future, this would be quite nice
                // :>>
                vy.scale(0.5),
                () -> yawFromNoteVelocity(calculateNoteVelocity(Seconds.of(0.2)))));
  }

  public static Pose2d robotPoseFacingSpeaker(Translation2d robotTranslation) {
    return new Pose2d(
        robotTranslation,
        translationToSpeaker(robotTranslation)
            .getAngle()
            .plus(Rotation2d.fromRadians(Math.PI / 2)));
  }

  public Vector<N3> calculateNoteVelocity() {
    return calculateNoteVelocity(drive.pose());
  }

  public Vector<N3> calculateNoteVelocity(Time predictionTime) {
    return calculateNoteVelocity(
        predictedPose(
            drive.pose(),
            drive.fieldRelativeChassisSpeeds(),
            predictionTime)); // add the chassis speeds methods for drive including get field
    // relative / robot relative etc
  }

  /**
   * Calculates a vector for the desired fuel velocity relative to the robot for it to travel into
   * the speaker, accounting for the robot's current motion.
   *
   * @return A 3d vector representing the desired fuel initial velocity.
   */
  public Vector<N3> calculateFuelVelocity(Pose2d robotPose) {
  /**
   * (1) It works
   * (2) It's correct
   * (3) It looks good and is readable (can add a bunch of other smaller components to the functions to, just so that it is readable through english)
   */

    ChassisSpeeds speeds = drive.fieldRelativeChassisSpeeds();

    Vector<N3> robotVelocity = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
    Translation2d difference = translationToSpeaker(robotPose.getTranslation());
    double shotVelo = calculateStationaryVelocity(difference.getNorm());

    //TODO make sure to look over this again next commit and get the correct values after understanding how exactly this translates to something different in Rebuilt2026
    Rotation3d fuelOrientation = 
      new Rotation3d(
          0,
          -calculateStationaryPitch(
            robotPoseFacingSpeaker(robotPose.getTranslation()),
            shotVelo, 
            0.0), 
          difference.getAngle().getRadians());

    //rotate unit forwar vector by fuel orientation and scale by our shot velocity
    Vector<N3> fuelVelocity = new Translation3d(1, 0, 0).rotateBy(fuelOrientation).toVector().unit().times(shotVelo);

    return fuelVelocity.minus(robotVelocity);
  }

  /**
   * 
   * Okay so ngl idk what happened here... I should probably organize it into something that I can actually physically read LMAO
   * 
   * ChassisSpeeds speeds = drive.getFieldRelativeChassisSpeeds(); 
   * 
   * Vector<N3> robotVelocity = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0); 
   * Translation2d difference = translationToSpeaker(robotPose.getTranslation()); 
   * double shotVelocity = calculateStationaryVelocity(difference.getNorm()); 
   * Rotation3d noteOrientation = new Rotation3d(0, -calculateStationaryPitch( robotPoseFacingSpeaker(robotPose.getTranslation()), shotVelocity, pivot.position()), //TODO, substitute with own function // change this block (eventually)
   * 
   * package this entire thing into a utility if possible) 
   * 
   * difference.getAngle().getRadians()); 
   * //rotate unit forward vector by note orientation and scale by our shot velocity 
   * Vector<N3> noteVelocity = new Translation3d(1, 0,
   * 0).rotateBy(noteOrientation).toVector().unit().times(shotVelocity);
   *
   * <p>return noteVelocity.minus(robotVelocity);
   */
  
  
   /** 
    * TODO Please write a JavaDoc for this.
    */
  
   public static Pose2d predictedPose(Pose2d robotPose, ChassisSpeeds speeds, Time predictionTime) {
    // TODO
  }

  /**
   * Vector<N3> current = VecBuilder.fill(robotPose.getX(), robotPose.getY(),
   * robotPose.getRotation().getRadians()); Vector<N3> velocity = VecBuilder.fill(
   * speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond); Vector<N3>
   * predicted = current.plus(velocity.times(predictionTime.in(Seconds))); return new
   * Pose2d(predicted.get(0), predicted.get(1), Rotation2d.fromRadians(predicted.get(2)));
   */

  /**
   * Returns the pose of the shooter.
   *
   * @return The pose of the shooter in 3d space.
   */
  
   @Logged
  public Pose3d shooterPose() {
    // TODO
  }

  /**
   * return new Pose3d(drive.pose()) .transformBy(pivot.transform())
   * .transformBy(PivotConstants.SHOOTER_FROM_AXLE);
   */
  public static Pose3d shooterPose(Transform3d pivot, Pose2d robot) {
    return new Pose3d(robot).transformBy(pivot).transformBy(PivotConstants.SHOOTER_FROM_AXLE);
  }

  /**
   * Calculates if the robot can make its current shot.
   *
   * @return Whether the robot can shoot from its current position at its current velocity.
   */
  @Logged
  public boolean inRange() { // ooohh! this is a pretty nice concept
    // TODO
  }

  /**
   * Vector<N3> shot = calculateNoteVelocity(); double pitch = pitchFromNoteVelocity(shot); return
   * MIN_ANGLE.in(Radians) < pitch && pitch < MAX_ANGLE.in(Radians) &&
   * Math.abs(rotationalVelocityFromNoteVelocity(shot)) < MAX_VELOCITY.in(RadiansPerSecond) &&
   * translationToSpeaker(drive.pose().getTranslation()).getNorm() < MAX_DISTANCE.in(Meters);
   */
  public boolean atYaw(Rotation2d yaw) {
    double tolerance = DriveConstants.Rotation.TOLERANCE.in(Radians) * (1 - yaw.getSin());
    Rotation2d diff = drive.heading().minus(yaw);
    return Math.abs(atan(diff.getTan())) < tolerance;
  }

  /**
   * Calculates pitch from note initial velocity vector. If given a robot relative initial velocity
   * vector, the return value will also be the pivot angle.
   *
   * @param velocity Note initial velocity vector
   * @return Pitch/pivot angle
   */
  public static double pitchFromNoteVelocity(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }

  /**
   * Calculates heading from note initial velocity vector. If given a robot relative initial
   * velocity vector, the return value will be the target robot heading.
   *
   * @param velocity Note initial velocity vector
   * @return Heading
   */
  public static Rotation2d yawFromNoteVelocity(Vector<N3> velocity) {
    return Rotation2d.fromRadians(Math.PI).plus(new Rotation2d(velocity.get(0), velocity.get(1)));
  }

  /**
   * Calculates magnitude of initial velocity vector of note, in radians per second. If given a
   * robot relative initial velocity vector, the return value will be the target flywheel speed
   * (ish).
   *
   * @param velocity Note initial velocity vector relative to the robot
   * @return Flywheel speed (rads / s)
   */
  public static double rotationalVelocityFromNoteVelocity(Vector<N3> velocity) {
    return velocity.norm() / RADIUS.in(Meters) * siggysConstant.get();
  }

  /**
   * Converts between flywheel speed and note speed
   *
   * @param flywheelSpeed Flywheel speed in radians per second
   * @return Note speed in meters per second
   */
  public static double flywheelToNoteSpeed(double flywheelSpeed) {
    return flywheelSpeed * RADIUS.in(Meters) / siggysConstant.get();
  }

  public static Translation2d translationToSpeaker(Translation2d robotTranslation) {
    return speaker().toTranslation2d().minus(robotTranslation);
  }

  public static double calculateStationaryVelocity(double distance) {
    return flywheelToNoteSpeed(shotVelocityLookup.get(distance));
  }

  /**
   * Calculates a stationary pitch from a pose so that the note goes into the speaker.
   *
   * @param shooterPose The pose of the shooter.
   * @param velocity The magnitude of velocity to launch the note at.
   * @return The pitch to shoot the note at.
   */
  public static double calculateStationaryPitch(
      Pose2d robotPose, double velocity, double prevPitch) {
    return calculateStationaryPitch(robotPose, velocity, prevPitch, 0);
  }

  private static double calculateStationaryPitch(
      Pose2d robotPose, double velocity, double prevPitch, int i) {
    double G = 9.81;
    Translation3d shooterTranslation =
        shooterPose(Pivot.transform(-prevPitch), robotPose).getTranslation();
    double dist = translationToSpeaker(shooterTranslation.toTranslation2d()).getNorm();
    double h = speaker().getZ() - shooterTranslation.getZ();
    double denom = (G * Math.pow(dist, 2));
    double rad =
        pow(dist, 2) * pow(velocity, 4)
            - G * pow(dist, 2) * (G * pow(dist, 2) + 2 * h * pow(velocity, 2));
    double pitch = Math.atan((1 / (denom)) * (dist * pow(velocity, 2) - Math.sqrt(rad)));
    if (Math.abs(pitch - prevPitch) < 0.005 || i > 50) {
      return pitch;
    }
    return calculateStationaryPitch(robotPose, velocity, pitch, i + 1);
  }
}
