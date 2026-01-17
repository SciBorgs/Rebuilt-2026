package org.sciborgs1155.robot.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretVisualizer {
  private final Mechanism2d mech;
  private final MechanismLigament2d positionArm;
  private final MechanismLigament2d setpointArm;

  public TurretVisualizer() {
    mech = new Mechanism2d(6, 7);

    MechanismRoot2d chassis = mech.getRoot("Chassis", 3, 3.5);

    positionArm =
        chassis.append(new MechanismLigament2d("Position", 3, 0, 4, new Color8Bit(0, 255, 0)));

    setpointArm =
        chassis.append(new MechanismLigament2d("Setpoint", 3, 0, 2, new Color8Bit(255, 0, 0)));

    SmartDashboard.putData("TurretVisualizer", mech);
  }

  public void setPosition(double angleRad) {
    positionArm.setAngle(Units.radiansToDegrees(angleRad));
  }

  public void setSetpoint(double angleRad) {
    setpointArm.setAngle(Units.radiansToDegrees(angleRad));
  }
}
