package org.sciborgs1155.robot.intake;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Intake implements AutoCloseable{
    //hardware initialization
    public static IntakeIO hardware;

    //TODO set up IntakeIO Constructor so this sequence works
    /**
     * (1) method signature (input: IntakeIO) (output: Intake class)
     * (2) this.hardware = hardware
     *
     */

    public Intake(IntakeIO hardware) {
        this.hardware = hardware;
    }

    public static Intake create(){
        if (Robot.isReal()){
            return new Intake(new RealIntake());
        }
        else{
            return new Intake(new SimIntake());
        }
    }

    //TODO 
    /**
     * 
     * Command Factory:
    
     * 
    */
    public Command extend(){
        return new InstantCommand(() -> hardware.extend());
    }

    public Command startRollers(){
        return Commands.run(() -> hardware.setRollerVoltage());
    }
    
    public Command retract(){
        return new InstantCommand(() -> hardware.retract());
    }

    @Override
    public void close() throws Exception {}


    /*public AngularVelocity rollerVelocity() {
        return hardware.rollerVelocity();
    }

    public void setRollerVoltage() {
        hardware.setRollerVoltage();
    }

    public void setExtensionVoltage() {
        hardware.setExtensionVoltage();
    }

    public double extensionPosition() {
        return hardware.extensionPosition();
    }*/

}
