package org.sciborgs1155.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Set;
import org.sciborgs1155.robot.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** TODO: Create cameras with updated constants; be sure to add in {@link Vision#create} */
  // WARNING: EMPTY TRANSFORMS WILL CRASH SIMULATION UPON TAG DETECTION
  public static final CameraConfig BACK_LEFT_CAMERA =
      new CameraConfig(
          "back left",
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))));

  public static final CameraConfig BACK_RIGHT_CAMERA =
      new CameraConfig(
          "back right",
          new Transform3d(
              Inches.of(1),
              Inches.of(1),
              Inches.of(1),
              new Rotation3d(Degrees.zero(), Degrees.of(-45), Degrees.zero())
                  .rotateBy(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)))));

  // ThriftyCam constants for our configuration
  public static final int WIDTH = 1280;
  public static final int HEIGHT = 720;
  public static final Rotation2d FOV = Rotation2d.fromDegrees(80);

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
