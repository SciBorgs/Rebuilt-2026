package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.FieldConstants.allianceReflect;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;
import static org.sciborgs1155.robot.shooter.ShooterConstants.IDLE_VELOCITY;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Function;
import java.util.function.Supplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer;
import org.sciborgs1155.robot.commands.shooting.ShootingAlgorithm;
import org.sciborgs1155.robot.commands.shooting.TOFIteration;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.hood.HoodConstants;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.ShooterConstants;
import org.sciborgs1155.robot.turret.Turret;

public class Shooting {
  /**
   * The time from when we command the shooter to when the ball is actually shot, in seconds. This
   * can be used to account for errors in the model, resulting from missing the shot while moving
   * (but not stationary).
   */
  public static final DoubleEntry LATENCY_TIME = Tuning.entry("/ShootingData/Latency Time", .1);

  public static final DoubleEntry RADS_TEST = Tuning.entry("/ShootingData/RADS", 100.0);
  public static final DoubleEntry HOOD_DEGREES_TEST =
      Tuning.entry("/ShootingData/Hood Angle", 30.0);

  public static final Distance MAX_DISTANCE = Meters.of(100);
  public static final Distance MIN_DISTANCE = Meters.of(.2);

  /** Field-relative position of the hub target. */
  public static final Translation2d HUB_TARGET = 
      allianceReflect(FieldConstants.Hub.TOP_CENTER_POINT.toTranslation2d());
  public static final Translation2d LEFT_FEED = allianceReflect(FieldConstants.Hub.LEFT_FEED.toTranslation2d());
  public static final Translation2d RIGHT_FEED = allianceReflect(FieldConstants.Hub.RIGHT_FEED.toTranslation2d());

  private final ShootingAlgorithm algorithm = new TOFIteration();

  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Drive drive;
  private final Hopper hopper;
  private final Indexer indexer;

  ProjectileVisualizer fuelVisualizer;

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
      ProjectileVisualizer fuelVisualizer) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.drive = drive;
    this.hopper = hopper;
    this.indexer = indexer;
    this.fuelVisualizer = fuelVisualizer;
  }

  /** Parameters to command the shooter superstructure. */
  public record ShooterParams(double RADS, double hoodAngle, double turretAngle) {}

  /**
   * Shoots the ball at the hub. Can do it while driving.
   *
   * @return a command that shoots at the hub while driving
   */
  public Command shootDriving(Translation2d target, InputStream vx, InputStream vy, InputStream omega) {
    return Commands.waitUntil(
            () ->
                shooter.atSetpoint()
                    && shooter.setpoint() > IDLE_VELOCITY.in(RadiansPerSecond)
                    && hood.atGoal())
        // && turret.atGoal())
        .andThen(
            Commands.parallel(
                hopper.intake(),
                indexer.forward(),
                Commands.runOnce(
                    () -> {
                      if (fuelVisualizer != null) fuelVisualizer.launchProjectile();
                    })))
        .deadlineFor(
            runShooterSuperstructure(() -> calculateShot(HUB_TARGET)),
            drive.drive(
                vx.scale(DriveConstants.SHOOTING_TRANSLATIONAL_SPEED),
                vy.scale(DriveConstants.SHOOTING_TRANSLATIONAL_SPEED),
                omega.scale(DriveConstants.SHOOTING_ANGULAR_SPEED)));
  }

  private Command runShooterSuperstructure(Supplier<ShooterParams> params) {
    return Commands.parallel(
        shooter.runShooter(() -> params.get().RADS),
        hood.goTo(() -> params.get().hoodAngle),
        turret.goTo(() -> params.get().turretAngle));
  }

  /**
   * @return A Command to move the hood all the way down and idle the shooter.
   */
  public Command hideAway() {
        return Commands.parallel(
            shooter.idle(),
            hood.goTo(HoodConstants.MIN_ANGLE)
        );
  }

  /**
   * @return A Trigger that activates when imminently crossing between field zones within HOOD_DOWN_TIME.
   */
  public Trigger crossingAlliance() {
    return new Trigger(() -> {
        double projectedDeltaX = drive.velocity().get(0) * HoodConstants.HOOD_DOWN_TIME.in(Seconds);
        double poseX = drive.pose().getX();
        double nearHubDisplacement = FieldConstants.LinesVertical.HUB_CENTER - poseX;
        double farHubDisplacement = FieldConstants.LinesVertical.OPP_HUB_CENTER - poseX;
        Function<Double,Boolean> compare = (Double displacement) -> Math.abs(projectedDeltaX) > Math.abs(displacement) && ((projectedDeltaX > 0) == (displacement > 0));
        return compare.apply(nearHubDisplacement) || compare.apply(farHubDisplacement);
    });
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
                    RADS_TEST.get(),
                    HOOD_DEGREES_TEST.get() * Math.PI / 180,
                    calculateShot(HUB_TARGET).turretAngle))
        .alongWith(fuelVisualizer != null ? fuelVisualizer.launchProjectiles() : Commands.none());
  }

  /**
   * Calculates a shot at the given target. Accounts for robot velocity and latency.
   *
   * @param target field-relative x/y position of the target
   * @return parameters to command the shooter superstructure to
   */
  public ShooterParams calculateShot(Translation2d target) {
    // Latency-compensated robot pose
    Pose2d latencyPose =
        drive.pose().exp(drive.robotRelativeChassisSpeeds().toTwist2d(LATENCY_TIME.get()));
    LoggingUtils.log("/ShootingData/Latency Pose", latencyPose, Pose2d.struct);

    // Turret position at the latency-compensated pose

    Pose2d turretPose =
        latencyPose.transformBy(
            new Transform2d(
                CENTER_TO_SHOOTER.getTranslation().toTranslation2d(),
                CENTER_TO_SHOOTER.getRotation().toRotation2d()));
    LoggingUtils.log("/ShootingData/Projected Turret Pose", turretPose, Pose2d.struct);

    // Field-relative turret velocity: translational + tangential from rotation
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

    // Displacement from turret to target
    Translation2d turretTranslation = turretPose.getTranslation();
    Translation3d displacement = new Translation3d(target.minus(turretPose.getTranslation()));

    // Run the shooting algorithm to get field-relative firing vector
    Vector<N3> firingVec = algorithm.calculate(displacement, turretSpeeds);

    double vx = firingVec.get(0);
    double vy = firingVec.get(1);
    double vz = firingVec.get(2);

    double rads = firingVec.norm();
    double hoodAngle = Math.atan2(vz, Math.hypot(vx, vy));
    double fieldYaw = Math.atan2(vy, vx);
    double turretAngle = fieldYaw - drive.pose().getRotation().getRadians();

    LoggingUtils.log("/ShootingData/Distance", turretTranslation.getDistance(target));

    return new ShooterParams(rads, hoodAngle, turretAngle);
  }
}
