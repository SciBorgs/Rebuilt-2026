package org.sciborgs1155.robot.climb;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.climb.ClimbConstants.MIN_HEIGHT;

import org.sciborgs1155.lib.LoggingUtils;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbVisualizer {
  @NotLogged private final Mechanism2d mech; // The physical field the robot is in
  private final MechanismLigament2d elevator; // The elevator mechanism
  private final String name;

  public ClimbVisualizer(String name, Color8Bit color) {
    this.name = name;
    mech = new Mechanism2d(50, 50);
    MechanismRoot2d chassis = mech.getRoot("chassis", 25, 10);
    elevator =
        chassis.append(
            new MechanismLigament2d("elevator", MIN_HEIGHT.in(Meters) * 10, 90, 3, color));
  }

  /**
   * Sets the length of the elevator visualization.
   *
   * @param length The length to set.
   */
  public void setLength(double length) {
    elevator.setLength(length * 10);
    SmartDashboard.putData("Robot/climb/" + name, mech);
  }
  
}
