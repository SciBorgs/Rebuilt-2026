package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;

/**
 * Source:
 * https://github.com/Mechanical-Advantage/AdvantageKit/blob/02b7b64d8ebe399c931458f906b544582b765df7/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/subsystems/drive/Drive.java
 * https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#wheel-radius-characterization
 */
public class DriveCommands {
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 1; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.25; // Rad/Sec^2
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  DriveConstants.MODULE_OFFSET[0].getMeasureX().in(Meters),
                  DriveConstants.MODULE_OFFSET[0].getMeasureY().in(Meters)),
              Math.hypot(
                  DriveConstants.MODULE_OFFSET[1].getMeasureX().in(Meters),
                  DriveConstants.MODULE_OFFSET[1].getMeasureY().in(Meters))),
          Math.max(
              Math.hypot(
                  DriveConstants.MODULE_OFFSET[2].getMeasureX().in(Meters),
                  DriveConstants.MODULE_OFFSET[2].getMeasureY().in(Meters)),
              Math.hypot(
                  DriveConstants.MODULE_OFFSET[3].getMeasureX().in(Meters),
                  DriveConstants.MODULE_OFFSET[3].getMeasureY().in(Meters))));

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.setChassisSpeeds(
                      new ChassisSpeeds(0, 0, speed), ControlMode.CLOSED_LOOP_VELOCITY);
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.heading();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.heading();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      FaultLogger.report(
                          "wheel characterization",
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches",
                          FaultType.INFO);
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
