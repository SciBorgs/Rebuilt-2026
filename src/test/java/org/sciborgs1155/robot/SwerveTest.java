package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.drive.NoGyro;
import org.sciborgs1155.robot.drive.SimModule;

/** Swerve test. Currently incomplete and does nothing. */
public class SwerveTest {
  SimModule frontLeft;
  SimModule frontRight;
  SimModule rearLeft;
  SimModule rearRight;
  NoGyro gyro;
  Drive drive;

  final double DELTA = 0.15;

  @BeforeEach
  public void setup() {
    setupTests();
    frontLeft = new SimModule("FL");
    frontRight = new SimModule("FR");
    rearLeft = new SimModule("RL");
    rearRight = new SimModule("RR");
    gyro = new NoGyro();
    drive = new Drive(gyro, frontLeft, frontRight, rearLeft, rearRight);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive);
  }

  @Disabled
  @Test
  public void systemCheck() {
    runUnitTest(drive.systemsCheck());
  }

  @RepeatedTest(5)
  public void reachesRobotVelocity() {
    double xVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;
    double yVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;

    run(drive.drive(() -> xVelocitySetpoint, () -> yVelocitySetpoint, () -> Rotation2d.kZero));
    fastForward(500);

    ChassisSpeeds chassisSpeed = drive.fieldRelativeChassisSpeeds();

    assertEquals(xVelocitySetpoint, chassisSpeed.vxMetersPerSecond, DELTA);
    assertEquals(yVelocitySetpoint, chassisSpeed.vyMetersPerSecond, DELTA);
  }

  @RepeatedTest(5)
  public void reachesAngularVelocity() {
    double omegaRadiansPerSecond = Math.random() * 2 - 1;
    run(
        drive.run(
            () ->
                drive.setChassisSpeeds(
                    new ChassisSpeeds(0, 0, omegaRadiansPerSecond),
                    ControlMode.CLOSED_LOOP_VELOCITY)));
    fastForward();

    ChassisSpeeds chassisSpeed = drive.robotRelativeChassisSpeeds();
    assertEquals(omegaRadiansPerSecond, chassisSpeed.omegaRadiansPerSecond, DELTA);
  }

  @RepeatedTest(value = 5, failureThreshold = 1)
  public void testModuleDistance() throws Exception {
    assertEquals(drive.pose().getX(), 0);
    assertEquals(drive.pose().getY(), 0);
    assertEquals(drive.pose().getRotation().getRadians(), 0);
    double xVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;
    double yVelocitySetpoint = Math.random() * (2 * 2.265) - 2.265;

    double deltaT = 4;
    double deltaX = xVelocitySetpoint * deltaT;
    double deltaY = yVelocitySetpoint * deltaT;

    String name = "test run " + Math.floor(Math.random() * 1000);

    Command c =
        drive
            .run(
                () ->
                    drive.setChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocitySetpoint, yVelocitySetpoint, 0, drive.heading()),
                        ControlMode.CLOSED_LOOP_VELOCITY))
            .withName(name);

    run(c);

    fastForward(Seconds.of(deltaT));

    assertEquals(drive.getCurrentCommand().getName(), name);

    Pose2d pose = drive.pose();

    assertEquals(deltaX, pose.getX(), DELTA * 4);
    assertEquals(deltaY, pose.getY(), DELTA * 4);
  }

  @Disabled
  @RepeatedTest(20)
  public void assistedDrivingTest() {
    Pose2d target =
        // new Pose2d(
        //     Math.random() * 10 + 2,
        //     Math.random() * 10 + 2,
        //     Rotation2d.fromRotations(Math.random()));
        new Pose2d(5, 5, Rotation2d.k180deg);

    Rotation2d offset = Rotation2d.fromRadians(/*Math.random() * 0.2 - 0.1*/ -0.05);
    Translation2d input =
        (target.getTranslation().rotateBy(offset)).div(target.getTranslation().getNorm());

    runToCompletion(
        drive
            .assistedDrive(input::getX, input::getY, () -> 0, target)
            .until(
                () ->
                    target.getTranslation().minus(drive.pose().getTranslation()).getNorm()
                        < Translation.TOLERANCE.in(Meters))
            .withTimeout(Seconds.of(20)));

    Translation2d velocities =
        new Translation2d(
            drive.fieldRelativeChassisSpeeds().vxMetersPerSecond,
            drive.fieldRelativeChassisSpeeds().vyMetersPerSecond);

    System.out.println("velocities: " + velocities);

    System.out.println(offset.getDegrees());
    System.out.println(velocities.getAngle());
    System.out.println(input.getAngle());

    assertTrue(offset.getSin() > 0 == velocities.getAngle().minus(input.getAngle()).getSin() > 0);

    assertEquals(drive.pose().getRotation().getSin(), target.getRotation().getSin(), 0.05);
  }
}
