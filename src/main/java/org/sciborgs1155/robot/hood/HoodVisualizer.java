package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HoodVisualizer {
  private final Mechanism2d mech;
  private final MechanismLigament2d hood;
  private final MechanismLigament2d fuelTrajectory;
  private final String name;

  /**
   * thr constructor for hood visualizer
   *
   * @param name name of the visualizer
   * @param hoodColor color of the hood
   * @param fuelTrajColor color of the line depicting the fuel trajectory
   */
  public HoodVisualizer(String name, Color8Bit hoodColor, Color8Bit fuelTrajColor) {
    this.name = name;
    mech = new Mechanism2d(100, 100);
    MechanismRoot2d chassis = mech.getRoot("Chassis", 50, 10);
    hood =
        chassis.append(
            new MechanismLigament2d(
                "hood", HOOD_RADIUS.in(Inches) * 5, STARTING_ANGLE.in(Degrees), 3, hoodColor));
    fuelTrajectory =
        hood.append(new MechanismLigament2d("Fuel Trajectory", 25, 90, 2, fuelTrajColor));
  }

  /**
   * sets the visualizer to a certain angle
   *
   * @param angleDegrees the angle to set the visualizer to in degrees
   */
  public void setAngle(double angleDegrees) {
    hood.setAngle(angleDegrees);
    SmartDashboard.putData("/Robot/hood/" + name, mech);
  }
}
