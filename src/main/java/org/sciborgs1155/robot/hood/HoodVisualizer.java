package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HoodVisualizer {
    private final Mechanism2d mech;
    private final MechanismLigament2d hood;
    private final String name;
    public HoodVisualizer(String name, Color8Bit color) {
        this.name = name;
        mech = new Mechanism2d(100, 100);
        MechanismRoot2d chassis = mech.getRoot("Chassis", 50, 10);
        hood = chassis.append(new MechanismLigament2d("hood", HOOD_RADIUS.in(Inches), STARTING_ANGLE.in(Degrees), 3, color));
        
    }
}
