package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
    /**
     * 
     * We still want this for later, just add on more later TODO
     * 
     * You probably what the angular to linear scaling factors in here based on the dimensions later
     * 
     *  */

    public static Current CURRENT_LIMIT = Amps.of(5);
    public static double ROLLER_VOLTAGE = 10;
    public static double EXTEND_VOLTAGE = 10;
}
