package org.sciborgs1155.robot.turret;

import static org.sciborgs1155.robot.turret.TurretConstants.VISUALIZER_HEIGHT;
import static org.sciborgs1155.robot.turret.TurretConstants.VISUALIZER_WIDTH;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Visualizes the {@code Turret} subsystem using {@code Mechanism2d}. */
public class TurretVisualizer {
  /** Contains all roots and ligaments of the visualizer. */
  private final Mechanism2d mechanism = new Mechanism2d(VISUALIZER_WIDTH, VISUALIZER_HEIGHT);

  /** Center of rotation for the ligaments. */
  private final MechanismRoot2d root =
      mechanism.getRoot("Chassis", VISUALIZER_WIDTH / 2, VISUALIZER_HEIGHT / 2);

  /** Visualizes the angular position of the {@code Turret}. */
  private final MechanismLigament2d position =
      root.append(new MechanismLigament2d("Position", 3, 0, 4, new Color8Bit(0, 255, 0)));

  /** Visualizes the angular position of the {@code Turret}'s target. */
  private final MechanismLigament2d target =
      root.append(new MechanismLigament2d("Target", 3, 0, 2, new Color8Bit(255, 0, 0)));

  private final MechanismLigament2d blindspotA =
      root.append(new MechanismLigament2d("Blindspot A", 3, 0, 2, new Color8Bit(255, 255, 0)));

  private final MechanismLigament2d blindspotB =
      root.append(new MechanismLigament2d("Blindspot B", 3, 0, 2, new Color8Bit(255, 255, 0)));

  /** Visualizes the {@code Turret} subsystem using {@code Mechanism2d}. */
  public TurretVisualizer(int width, int height) {
    SmartDashboard.putData("TurretVisualizer", mechanism);
    blindspotA.setAngle(175);
    blindspotB.setAngle(-175);
  }

  /**
   * Updates the visualizer. To be called periodically.
   *
   * @param positionRadians The angular position of the turret.
   * @param setpointRadians The angular setpoint of the turret (radians).
   */
  public void update(double positionRadians, double targetRadians) {
    position.setAngle(Units.radiansToDegrees(positionRadians));
    target.setAngle(Units.radiansToDegrees(targetRadians));
  }
}
