package org.sciborgs1155.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Set;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public final class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // The PoseStrategy in multitag mode when only one tag is seen. Do NOT use MULTI_TAG_PNP varients.
  public static final PoseStrategy SINGLE_TAG_FALLBACK = PoseStrategy.LOWEST_AMBIGUITY;

  /** TODO: Create cameras with updated constants; be sure to add in {@link Vision#create} */
  // WARNING: EMPTY TRANSFORMS WILL CRASH SIMULATION UPON TAG DETECTION
  // TODO: actually add camera positions, figure out if its actually 148 fov
  public static final CameraConfig FL_CAMERA = // CAMERA 0
      new CameraConfig(
          "FL cam",
          78,
          new Transform3d(
              Inches.of(11.265),
              Inches.of(12.617898),
              Inches.of(9.232183),
              yawPitchRoll(90, -20, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  public static final CameraConfig FR_CAMERA = // CAMERA 1
      new CameraConfig(
          "FR cam",
          78,
          new Transform3d(
              Inches.of(11.265),
              Inches.of(-12.617898),
              Inches.of(9.232183),
              yawPitchRoll(-90, -20, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  public static final CameraConfig F_CAMERA = // CAMERA 4
      new CameraConfig(
          "F cam",
          78,
          new Transform3d(
              Inches.of(13.514), Inches.of(2.699), Inches.of(10.581108), yawPitchRoll(0, 15, 0)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  public static final CameraConfig B_CAMERA = // CAMEwRA 2
      new CameraConfig(
          "B cam",
          78,
          new Transform3d(
              Inches.of(1.827756),
              Inches.of(-3.312500),
              Inches.of(20.928461),
              yawPitchRoll(180, 0, 180)),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  // Camera constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
      VecBuilder.fill(0.9, 0.9, 1155); // TODO decide these later when we test the bump
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.6, 0.6, 1155);
  public static final Matrix<N3, N1> SUPERTRUST_TAG_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.01);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = Math.PI;
  public static final double MAX_AMBIGUITY = 0.20;
  public static final double MAX_DISTANCE = FieldConstants.FIELD_LENGTH / 2;

  // Total of 32 AprilTags
  // Reference: https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf (page 33)
  // Red Climb 15, 16
  // Blue Climb 31, 32
  // Red Hub 3, 4, 5, 8, 9, 10, 11
  // Blue Hub 18, 19, 20, 21, 24, 25, 26, 27
  // Red Trenches 6, 7, 12, 1
  // Blue Trenches 22, 23, 28, 17
  // Red Outpost 13, 14
  // Blue Outpost 29, 30

  public static final double[] TAG_WEIGHTS = {
    1, 1, .5, 1, 1,
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    1, 1
  };

  public static final Set<Integer> UNREPUTABLE_TAGS = Set.of(29);

  // Prevents instantiation
  private VisionConstants() {}

  /**
   * Returns a {@link Rotation3d} that represents a camera rotation, given the yaw, pitch, and roll.
   */
  public static Rotation3d yawPitchRoll(
      double yawDegrees, double pitchDegrees, double rollDegrees) {
    return new Rotation3d(
        Degrees.of(rollDegrees), Degrees.of(pitchDegrees), Degrees.of(yawDegrees));
  }
}
