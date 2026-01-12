package org.sciborgs1155.robot.hood;


import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{

    private final HoodIO hardware;

    public Hood(HoodIO hardware) {
        this.hardware = hardware;
    }

    public static Hood create() {
        return new Hood(Robot.isReal() ? new RealHood() : new SimHood());
    }
    
    public static Hood none() {
        return new Hood(new NoHood());
    }
    
}
