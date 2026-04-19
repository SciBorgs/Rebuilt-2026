package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.lib.ProjectileVisualizer.X;
import static org.sciborgs1155.lib.ProjectileVisualizer.Y;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.CALIBRATION_ENTRIES;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.CALIBRATION_INCREMENT;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.HOOD_ANGLE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.ROLLER_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.CalibrationConstants.STARTING_ROLLER_SPEED;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants.MIN_DISTANCE;
import static org.sciborgs1155.robot.commands.shooting.ShootingConstants.ScoringConstants.GOAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.ParameterLookupConstants.LookupID;
import org.sciborgs1155.robot.commands.shooting.ShootingConstants.PhysicalConstants;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.slapdown.Slapdown;
import org.sciborgs1155.robot.turret.Turret;

/** A command factory for shooter calibration. */
@SuppressWarnings("PMD.FieldNamingConventions")
public class Calibrator {
  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Hopper hopper;
  private final Indexer indexer;
  private final Slapdown slapdown;
  private final Drive drive;

  /** Whether the shooter is running or not. */
  private boolean shooting;

  /** The results of the calibration. */
  private static double[][] calibrationResults = new double[CALIBRATION_ENTRIES][3];

  /** The speed of the shooter roller in radians per second. */
  private static final DoubleEntry shooterSpeed =
      Tuning.entry("Shooting/Calibrator/TEST RADS", STARTING_ROLLER_SPEED);

  /** The index of the calibration. */
  private static final IntegerEntry index = Tuning.entry("Shooting/Calibrator/INDEX", 0);

  /** A command factory for shooter calibration. */
  public Calibrator(
      Shooter shooter,
      Turret turret,
      Hood hood,
      Hopper hopper,
      Indexer indexer,
      Slapdown slapdown,
      Drive drive) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.hopper = hopper;
    this.indexer = indexer;
    this.slapdown = slapdown;
    this.drive = drive;
  }

  /** Records a calibration result. */
  public Command recordCalibration() {
    return Commands.runOnce(
        () -> {
          if (index.get() < 0 || index.get() > CALIBRATION_ENTRIES)
            throw new UnsupportedOperationException("Invalid calibration index!");
          calibrationResults[(int) index.get()][DISTANCE] = distance(index.get());
          calibrationResults[(int) index.get()][ROLLER_SPEED] = shooterSpeed.get();
          calibrationResults[(int) index.get()][HOOD_ANGLE] = hoodAngle().getAsDouble();
        });
  }

  /** Runs the shooter, hopper and indexer; launches FUEL. */
  public Command runShooter() {
    return shooter
        .runShooter(shooterSpeed::get)
        .until(shooter::atSetpoint)
        .andThen(
            hopper
                .intake()
                .alongWith(indexer.forward())
                .alongWith(slapdown.squeeze())
                .alongWith(Commands.startEnd(() -> shooting = true, () -> shooting = false)))
        .withName("run shooter");
  }

  /** Prepares the hood, turret, and drive for calibration. */
  public Command prepareCalibration() {
    return Commands.parallel(
            drive.follow(calibrationPose()),
            hood.goTo(hoodAngle()),
            turret.goToYaw(Rotation2d.fromRadians(0)))
        .withName("prepare calibration");
  }

  /** Whether or not the hood, turret, and drive are at setpoint. */
  public Trigger readyForCalibration() {
    return new Trigger(
        () -> hood.atGoal() && turret.atGoal() && drive.atPose(calibrationPose().get()));
  }

  /** Logs shooting data to NetworkTables. */
  public void updateLogging() {
    LoggingUtils.log("Shooting/Calibrator/Ready", readyForCalibration().getAsBoolean());
    LoggingUtils.log("Shooting/Calibrator/Shooting", shooting);
    LoggingUtils.log(
        "Shooting/Calibrator/Calibration Pose", calibrationPose().get(), Pose2d.struct);

    for (int index = 0; index < calibrationResults.length; index++)
      LoggingUtils.log(
          "Shooting/Calibrator/Results/Calibration" + index, calibrationResults[index]);
  }

  /** Returns the calibration pose for the specific calibration index. */
  private Supplier<Pose2d> calibrationPose() {
    return () ->
        new Pose2d(
            GOAL[X] - distance(index.get()) - PhysicalConstants.ROBOT_TO_SHOOTER[X],
            GOAL[Y] - PhysicalConstants.ROBOT_TO_SHOOTER[Y],
            Rotation2d.kZero);
  }

  /** A supplier for the angle of the hood. */
  private DoubleSupplier hoodAngle() {
    ParameterLookup.useLookup(LookupID.MINIMAL_AIR_TIME);
    return () -> Math.PI / 2 - ParameterLookup.pitch(distance(index.get()));
  }

  /**
   * Returns the distance from the HUB at a specific calibration index.
   *
   * @param index the index of the calibration
   */
  private static double distance(long index) {
    if (index < 0 || index > CALIBRATION_ENTRIES)
      throw new UnsupportedOperationException("Invalid calibration index!");
    return MIN_DISTANCE + CALIBRATION_INCREMENT * index;
  }
}
