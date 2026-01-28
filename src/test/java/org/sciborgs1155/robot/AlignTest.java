package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.runToCompletion;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.vision.VisionConstants.yawPitchRoll;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;

public class AlignTest {
  Drive drive;
  Alignment align;

  /** Sets up the test environment and initializes drive and alignment before each test. */
  @BeforeEach
  public void setup() {
    setupTests();
    drive = Drive.create();
    drive.resetEncoders();
    drive.resetOdometry(Pose2d.kZero);
    align = new Alignment(drive);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  /** Cleans up resources after each test. */
  @AfterEach
  public void destroy() throws Exception {
    reset(drive);
  }

  /** Tests whether the obstacle-avoiding pathing works correctly. */
  @Test
  public void pathfindTest() throws Exception {
    Pose2d pose = new Pose2d(Meters.of(1), Meters.of(1), Rotation2d.kZero);
    // Make and run the pathfinding command
    runToCompletion(align.alignTo(() -> pose).withTimeout(Seconds.of(20)));

    // Assert the command works
    assertEquals(pose.getX(), drive.pose().getX(), Translation.TOLERANCE.in(Meters));
    assertEquals(pose.getY(), drive.pose().getY(), Translation.TOLERANCE.in(Meters));
    assertEquals(
        0,
        pose.getRotation().minus(drive.pose().getRotation()).getRadians(),
        Rotation.TOLERANCE.in(Radians));
  }

  /** Tests whether yawpitchroll works. Honestly this is completely redundant but it gives me joy */
  @RepeatedTest(10)
  public void yawPitchRollTest() throws Exception {
    Angle yaw = Degrees.of(360 * Math.random());
    Angle pitch = Degrees.of(360 * Math.random());
    Angle roll = Degrees.of(360 * Math.random());

    Rotation3d ypr = yawPitchRoll(yaw.in(Degrees), pitch.in(Degrees), roll.in(Degrees));

    Rotation3d def = new Rotation3d(roll, pitch, yaw);

    assertEquals(ypr, def);
  }
}
