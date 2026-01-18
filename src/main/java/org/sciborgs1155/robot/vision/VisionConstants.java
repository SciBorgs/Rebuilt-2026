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
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // The PoseStrategy in multitag mode when only one tag is seen. Do NOT use MULTI_TAG_PNP varients.
  public static final PoseStrategy SINGLE_TAG_FALLBACK = PoseStrategy.LOWEST_AMBIGUITY;

  /** TODO: Create cameras with updated constants; be sure to add in {@link Vision#create} */
  // WARNING: EMPTY TRANSFORMS WILL CRASH SIMULATION UPON TAG DETECTION
  // TODO: actually add camera positions, figure out if its actually 148 fov
  public static final CameraConfig CAMERA_0 =
      new CameraConfig(
          "cam 0 RENAME",
          78,
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  public static final CameraConfig CAMERA_1 =
      new CameraConfig(
          "cam 1 RENAME",
          78,
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  public static final CameraConfig CAMERA_2 =
      new CameraConfig(
          "cam 2 RENAME",
          78,
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  public static final CameraConfig CAMERA_3 =
      new CameraConfig(
          "cam 3 RENAME",
          148,
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  public static final CameraConfig CAMERA_4 =
      new CameraConfig(
          "cam 4 RENAME",
          148,
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  public static final CameraConfig CAMERA_5 =
      new CameraConfig(
          "cam 5 RENAME",
          148,
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

  // Camera constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 1;
  public static final double MAX_AMBIGUITY = 0.18;

  /** TODO: Modify AprilTag information as needed. */
  // Total of n AprilTags
  // Reference:
  // Tag Locations (1-n) | Description...

  public static final double[] TAG_WEIGHTS = {
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1
  };

  public static final Set<Integer> REPUTABLE_TAGS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 22);
}
