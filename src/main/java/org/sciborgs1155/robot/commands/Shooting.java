package org.sciborgs1155.robot.commands;

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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.shooter.Shooter;

import static java.lang.Math.atan;


public class Shooting {
  private final Shooter shooter;
  private final Drive drive;
  // private final Hood hood;
  // private final Turret turret;
  // private final Hopper hopper; // all of this stuff is still waiting to be merged in

  /* Create Lookup Table */
  private static final InterpolatingDoubleTreeMap shotVelocityLookup = new InterpolatingDoubleTreeMap(); //I still don't really know how to use or construct this...


  public Shooting(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    // this.hood = hood;
    // this.turret = turret;
    // this.hopper = hopper;
  }

  /**
   * TODO
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity in radians per second to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(AngularVelocity desiredVelocity) {
    return null;
  }

  /**
   * TODO
   * Runs shooter to desired velocity, runs feeder once it reaches its velocity and
  shootCondition
   * is true.
   *
   * @param desiredVelocity Target velocity for the flywheel.
   * @param shootCondition Condition after which the feeder will run.
   */
  public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
    return Commands.waitUntil(
            () ->
                shooter.atVelocity(desiredVelocity.getAsDouble()) &&
  shootCondition.getAsBoolean())
        .andThen(hopper.eject()) //change this line for hopper instead TODO
        .deadlineWith(shooter.runShooter(desiredVelocity));
  }


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
            () -> rotationalVelocityFromFuelVelocity(calculateFuelVelocity()),
            () -> atYaw(yawFromNoteVelocity(calculateFuelVelocity()))) //atYaw --> this will be a turret function w/ a tolerance. might need to have separate functions for when the robot moves horizontally relative to hub
        .deadlineFor(
            drive.drive(
                vx.scale(
                    0.5), // see if we could speed this up in the future, this would be quite nice
                // :>>
                vy.scale(0.5),
                () -> yawFromNoteVelocity(calculateFuelVelocity(Seconds.of(0.2)))));
  }

  public static Pose2d robotPoseFacingHub(Translation2d robotTranslation) { //TODO speaker --> hub 
    return new Pose2d(
        robotTranslation,
        translationToHub(robotTranslation)
            .getAngle()
            .plus(Rotation2d.fromRadians(Math.PI / 2))); //why is this angle being added here?
  }

  public boolean atYaw(Rotation2d yaw) { //turret command instead of robot positioning

    double tolerance = DriveConstants.Rotation.TOLERANCE.in(Radians) * (1 - yaw.getSin()); // figure out the math over here and replace with a smaller prive funciton
    Rotation2d error = drive.heading().minus(yaw);

    return Math.abs(atan(error.getTan())) < tolerance;
  }


  public Vector<N3> calculateFuelVelocity() {
    return calculateFuelVelocity(drive.pose());
  }

  public Vector<N3> calculateFuelVelocity(Time predictionTime) { //this code structure seems to account for the extra yaw / pitch that I was looking for before 
    return calculateFuelVelocity(
        predictedPose(
            drive.pose(),
            drive.fieldRelativeChassisSpeeds(),
            predictionTime)); 
  }

  /**
   * Calculates a vector for the desired fuel velocity relative to the robot for it to travel
  into
   * the speaker, accounting for the robot's current motion.
   *
   * @return A 3d vector representing the desired fuel initial velocity.
   */
  public Vector<N3> calculateFuelVelocity(Pose2d robotPose) { //is it necessary at all to have the orientation of the fuel on hand? I don't understand this part as much

    /* Field Relative Speeds */
    ChassisSpeeds speeds = drive.fieldRelativeChassisSpeeds();

    /* Robot Velocity Vector */
    Vector<N3> robotVelocity = VecBuilder.fill(speeds.vxMetersPerSecond,
  speeds.vyMetersPerSecond, 0);
    
  /* The error to be filled */
  Translation2d difference = translationToHub(robotPose.getTranslation());
  
  /*  */
  double shotVelo = calculateStationaryVelocity(difference.getNorm()); //can we not just get the norm in all cases ? Is this ever useful for any type of orientation thingy

    //TODO make sure to look over this again next commit and get the correct values 
    Rotation3d fuelOrientation =
      new Rotation3d(
          0,
          -calculateStationaryPitch(
            robotPoseFacingHub(robotPose.getTranslation()), //why is this negative again?
            shotVelo,
            0.0), /* pitch */
          difference.getAngle().getRadians()); /* yaw */ 

    //rotate unit forwar vector by fuel orientation and scale by our shot velocity
    Vector<N3> fuelVelocity = new Translation3d(1, 0,
  0).rotateBy(fuelOrientation).toVector().unit().times(shotVelo);

    return fuelVelocity.minus(robotVelocity);
  }


   public static Pose2d predictedPose(Pose2d robotPose, ChassisSpeeds speeds, Time
  predictionTime) {
    // TODO (look at from crescendo again)

    Vector<N3> position = //is there a better name that could be used here?
      VecBuilder.fill(robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians());

    Vector<N3> velocity =
      VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

    Vector<N3> predicted = position.plus(velocity.times())
    
  }

  /**
   * Calculates pitch from note initial velocity vector. If given a robot relative initial
  velocity
   * vector, the return value will also be the hood angle.
   *
   * @param velocity Note initial velocity vector
   * @return Hood angle
   */
  public static double pitchFromNoteVelocity(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
    /* (z / x) /  y  */ // this is the formula for angle between rho and the base vector 
  }

  /**
   * Calculates heading from note initial velocity vector. If given a robot relative initial
   * velocity vector, the return value will be the target robot heading.
   *
   * @param velocity Note initial velocity vector
   * @return Heading
   */
  public static Rotation2d yawFromNoteVelocity(Vector<N3> velocity) {
    return Rotation2d.fromRadians(Math.PI).plus(new Rotation2d(velocity.get(0),
  velocity.get(1)));
  }

  /**
   * Calculates magnitude of initial velocity vector of note, in radians per second. If given a
   * robot relative initial velocity vector, the return value will be the target flywheel speed
   * (ish).
   *
   * @param velocity Note initial velocity vector relative to the robot
   * @return Flywheel speed (rads / s)
   */
  public static double rotationalVelocityFromFuelVelocity(Vector<N3> velocity) {
    return velocity.norm() / RADIUS.in(Meters) * siggysConstant.get(); //make some constants and valuess to reflect, see if we want to still honor siggysConstant? 
  }

  public static Translation2d translationToHub(Translation2d robotTranslation) {
    return hub().toTranslation2d().minus(robotTranslation); //TODO speaker --> make into hub --> ankit did set those things up which is very commendable and i thank him for that
  }

  public static double calculateStationaryVelocity(double distance) { //TODO HOOD Angle is flexible as well
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
        shooterPose(Pivot.transform(-prevPitch), robotPose).getTranslation(); //TODO pivot needs to be set as Turret instead --> look at the code that people are using there and try to make it work
    double dist = translationToSpeaker(shooterTranslation.toTranslation2d()).getNorm();
    double h = speaker().getZ() - shooterTranslation.getZ(); //TODO hub
    double denom = (G * Math.pow(dist, 2));
    double rad =
        Math.pow(dist, 2) * Math.pow(velocity, 4)
            - G * Math.pow(dist, 2) * (G * Math.pow(dist, 2) + 2 * h * Math.pow(velocity, 2)); //TODO try to import pow statically, this is bugging me
    double pitch = Math.atan((1 / (denom)) * (dist * Math.pow(velocity, 2) - Math.sqrt(rad)));
    if (Math.abs(pitch - prevPitch) < 0.005 || i > 50) {
      return pitch;
    }
    return calculateStationaryPitch(robotPose, velocity, pitch, i + 1);
  }
}
