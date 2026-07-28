package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.commands.Shooting.HUB_TARGET;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.commands.Shooting.ShooterParams;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.NoGyro;
import org.sciborgs1155.robot.drive.SimModule;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.hopper.Hopper;
import org.sciborgs1155.robot.indexer.Indexer;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.slapdown.Slapdown;
import org.sciborgs1155.robot.turret.Turret;

public class ShootingTest {
  private Drive drive;
  private Shooting shooting;

  private static final double TOLERANCE = 0.01;

  /** Sets up sim subsystems and the shooting command factory before each test. */
  @BeforeEach
  public void setup() {
    setupTests();
    drive =
        new Drive(
            new NoGyro(),
            new SimModule("FL"),
            new SimModule("FR"),
            new SimModule("RL"),
            new SimModule("RR"));
    shooting =
        new Shooting(
            Shooter.none(),
            Turret.none(),
            Hood.none(),
            drive,
            Hopper.none(),
            Indexer.none(),
            Slapdown.none(),
            new FuelVisualizer(
                () -> new double[] {0, 0, 0},
                () -> new double[] {0, 0, 0},
                () -> new double[] {0, 0, 0},
                () -> 0));
  }

  /** Cleans up resources after each test. */
  @AfterEach
  public void destroy() throws Exception {
    reset(drive);
  }

  /**
   * For a stationary robot at the origin, the turret angle should point directly at HUB_TARGET,
   * with no lead compensation needed.
   */
  @Test
  public void stationaryTurretAngle() {
    Translation2d turretPos = new Translation2d(CENTER_TO_SHOOTER.getX(), CENTER_TO_SHOOTER.getY());
    Translation2d displacement = HUB_TARGET.minus(turretPos);
    double expectedTurretAngle = Math.atan2(displacement.getY(), displacement.getX());

    ShooterParams params = shooting.calculateShot(HUB_TARGET);

    assertEquals(expectedTurretAngle, params.turretAngle(), TOLERANCE);
  }

  /**
   * For a stationary robot, the interpolated rads and hood angle should fall within the bounds
   * defined by the lookup table.
   */
  @Test
  public void stationaryLookupBounds() {
    ShooterParams params = shooting.calculateShot(HUB_TARGET);

    assertTrue(params.RADS() >= 125.0 && params.RADS() <= 200.0);
    assertTrue(
        params.hoodAngle() >= Math.toRadians(15) && params.hoodAngle() <= Math.toRadians(40));
  }

  /**
   * When the robot is moving, the turret angle should differ from the stationary case due to lead
   * shot compensation.
   */
  @Test
  public void movingRobotLeadCompensation() {
    ShooterParams stationary = shooting.calculateShot(HUB_TARGET);

    run(
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    new ChassisSpeeds(2.0, 0, 0), ControlMode.CLOSED_LOOP_VELOCITY)));
    fastForward(Seconds.of(1));

    ShooterParams moving = shooting.calculateShot(HUB_TARGET);

    assertNotEquals(stationary.turretAngle(), moving.turretAngle(), TOLERANCE);
  }

  /**
   * Simulates a stationary shot at HUB_TARGET using the full projectile physics and asserts the
   * ball scores. This validates that the lookup table and algorithm produce a shot that actually
   * makes it in.
   */
  @Disabled
  @Test
  public void stationaryShot() {
    ShooterParams params = shooting.calculateShot(HUB_TARGET);
    Pose3d robotPose = new Pose3d(drive.pose());

    double[] shotVel =
        FuelVisualizer.shotVelocity(
            params.RADS(), params.hoodAngle(), params.turretAngle(), robotPose);

    FuelVisualizer visualizer =
        new FuelVisualizer(() -> shotVel, () -> robotPose, ChassisSpeeds::new);
    visualizer.generateTrajectory();

    assertTrue(visualizer.willScore());
  }
}
