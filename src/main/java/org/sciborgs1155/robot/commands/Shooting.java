package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import java.util.function.Supplier;

import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.lib.projectiles.FuelVisualizer;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.turret.Turret;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Shooting {
  /**
   * The time from when we command the shooter to when the ball is actually shot, in seconds. This
   * can be used to account for errors in the model, resulting from missing the shot while moving
   * (but not stationary).
   */
  public static final DoubleEntry LATENCY_TIME = Tuning.entry("/ShootingData/Latency Time", .1);

  public static final DoubleEntry RADS_TEST = Tuning.entry("/ShootingData/RADS", 100.0);
  public static final DoubleEntry HOOD_DEGREES_TEST = Tuning.entry("/ShootingData/Hood Angle", 30.0);

  public static final Distance MAX_DISTANCE = Meters.of(7);
  public static final Distance MIN_DISTANCE = Meters.of(.2);

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_RADS =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_TOF =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  static {
    // These are simulated values, not real-life
    DISTANCE_TO_HOOD_ANGLE.put(6.122, Rotation2d.fromDegrees(40));
    DISTANCE_TO_TOF.put(6.122, 1.0);
    DISTANCE_TO_RADS.put(6.122, 270.0);

    DISTANCE_TO_HOOD_ANGLE.put(3.333, Rotation2d.fromDegrees(30));
    DISTANCE_TO_TOF.put(3.333, .8);
    DISTANCE_TO_RADS.put(3.333, 220.0);

    DISTANCE_TO_HOOD_ANGLE.put(1.184, Rotation2d.fromDegrees(15));
    DISTANCE_TO_TOF.put(1.184, .7);
    DISTANCE_TO_RADS.put(1.184, 190.0);
  }

  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Drive drive;
  private final Hopper hopper;
  private final Indexer indexer;

  FuelVisualizer fuelVisualizer;

  /**
   * Creates the shooting command factory with all subsystems passed in. Subsystems provide
   * information by composition, and are passed to make sure they aren't made in more than one
   * place.
   */
  public Shooting(
      Shooter shooter,
      Turret turret,
      Hood hood,
      Drive drive,
      Hopper hopper,
      Indexer indexer,
      FuelVisualizer fuelVisualizer) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.drive = drive;
    this.hopper = hopper;
    this.indexer = indexer;
    this.fuelVisualizer = fuelVisualizer;
  }

  // Constants I want somewhere else

  public record ShooterParams(double RADS, double hoodAngle, double turretAngle) {}

  private static final Translation2d HUB_TARGET =
      FieldConstants.Hub.TOP_CENTER_POINT.toTranslation2d();

  /**
   * Shoots the ball at the hub. Can do it while driving.
   *
   * @return
   */
  public Command shootHub() {
    return Commands.waitUntil(() -> shooter.atSetpoint() && hood.atGoal() && turret.atGoal())
        .andThen(hopper.intake().alongWith(indexer.forward()))
        .deadlineFor(runShooterSuperstructure(() -> calculateShot(HUB_TARGET))).alongWith(Commands.repeatingSequence(fuelVisualizer.launchProjectile()));
  }

  private Command runShooterSuperstructure(Supplier<ShooterParams> params) {
    return Commands.parallel(
        shooter.runShooter(() -> params.get().RADS),
        hood.goTo(() -> params.get().hoodAngle),
        turret.goTo(() -> params.get().turretAngle));
  }

  /**
   * Lets you drive around while the turret aims at the hub. Should be doing this most of the match.
   */
  public Command faceHub() {
    return turret.goTo(() -> calculateShot(HUB_TARGET).turretAngle);
  }

  /**
   * This lets us set parameters manually when we're gathering data to shoot. For each distance, we
   * adjust the velocity and angle until it goes in, and record it in the lookup tables.
   */
  public Command shootWithTestData() {
    return runShooterSuperstructure(
            () ->
                new ShooterParams(
                    RADS_TEST.get(), HOOD_DEGREES_TEST.get() * Math.PI / 180, calculateShot(HUB_TARGET).turretAngle))
        .alongWith(Commands.repeatingSequence(fuelVisualizer.launchProjectile()));
  }

  /**
   * Calculates a shot for any x, y, z of hub. Because our shooting is data driven, the z can't
   * change, but it can still be used for feeding
   *
   * @param target the x and y location of the target (with hub height)
   * @return the parameters to command relevant subsystems to
   */
  private ShooterParams calculateShot(Translation2d target) {
    Pose2d turretPose = projectTurretPose(target);
    LoggingUtils.log("/ShootingData/Turret Pose", turretPose, Pose2d.struct);
    double distance = turretPose.getTranslation().getDistance(target);
    LoggingUtils.log("/ShootingData/Distance", distance);

    double turretAngle = turretPose.getRotation().getRadians();
    LoggingUtils.log("/ShootingData/Turret Angle", turretAngle * 180 / Math.PI);
    double hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(distance).getRadians();
    double RADS = DISTANCE_TO_RADS.get(distance);

    return new ShooterParams(RADS, hoodAngle, turretAngle);
  }

  /**
   * Projects the turret pose forward in time using the ToF as a dt. Allows us to preform a static
   * shot from this new position to account for velocity on the ball.
   *
   * @return the projected turret pose.
   */
  private Pose2d projectTurretPose(Translation2d target) {
    ChassisSpeeds relativeSpeeds = drive.robotRelativeChassisSpeeds();
    Pose2d latencyPose =
        drive
            .pose()
            .exp(
                new Twist2d(
                    relativeSpeeds.vxMetersPerSecond * LATENCY_TIME.get(),
                    relativeSpeeds.vyMetersPerSecond * LATENCY_TIME.get(),
                    relativeSpeeds.omegaRadiansPerSecond * LATENCY_TIME.get()));
    LoggingUtils.log("/ShootingData/Latency Pose", latencyPose, Pose2d.struct);
    Pose2d turretPose =
        latencyPose.transformBy(
            new Transform2d(
                CENTER_TO_SHOOTER.getX(),
                CENTER_TO_SHOOTER.getY(),
                CENTER_TO_SHOOTER.getRotation().toRotation2d()));

    Pose2d lookAhead = turretPose;

    ChassisSpeeds speeds = drive.fieldRelativeChassisSpeeds();

    Vector<N2> translationSpeeds =
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Vector<N2> rotationSpeeds =
        CENTER_TO_SHOOTER
            .getTranslation()
            .toTranslation2d()
            .rotateBy(Rotation2d.kCCW_90deg.plus(drive.heading()))
            .toVector()
            .times(speeds.omegaRadiansPerSecond);
    Vector<N2> turretSpeeds = translationSpeeds.plus(rotationSpeeds);
    // ToF iterative solver
    for (int i = 0; i < 25; i++) {
      double distance = target.getDistance(lookAhead.getTranslation());
      double tof = DISTANCE_TO_TOF.get(distance);
      Translation2d projection = turretPose.getTranslation().plus(new Translation2d(tof * turretSpeeds.get(0), tof * turretSpeeds.get(1)));
      lookAhead =
          new Pose2d(
              projection,
              target.minus(projection).getAngle().minus(drive.pose().getRotation()));
    }

    return lookAhead;
  }
}
