package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;

import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.intake.IntakeConstants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import org.sciborgs1155.lib.FaultLogger;

public class RealIntake implements IntakeIO, AutoCloseable{
    public static TalonFX leftMotor;
    public static TalonFX rightMotor;
    public static TalonFX extensionMotor;

    public RealIntake() {
        //leftMotor = new TalonFX(-1);
        rightMotor = new TalonFX(-2);
        extensionMotor = new TalonFX(-3);

        //Rollers - Kraken X44
        //Check how integrated encoder works

        //TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration(); 
        TalonFXConfiguration extensionMotorConfig = new TalonFXConfiguration();

        //rightMotor.ControlMode

        //leftMotorConfig.clone(rightMotorConfig)

        //TODO Define confi
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT.in(Amps);

        rightMotor.getConfigurator().apply(rightMotorConfig);
        //leftMotor.getConfigurator().apply(rightMotorConfig);

        extensionMotor.getConfigurator().apply(rightMotorConfig);

        //leftMotor.setControl(new Follower(-2, MotorAlignmentValue.Opposed));

        TalonUtils.addMotor(rightMotor);
        //TalonUtils.addMotor(leftMotor);
        TalonUtils.addMotor(extensionMotor);

        FaultLogger.register(rightMotor);
        //FaultLogger.register(leftMotor);
        FaultLogger.register(extensionMotor);
    }

    @Override
    public AngularVelocity rollerVelocity() {
        return rightMotor.getVelocity().getValue();
    }

    @Override
    public void setRollerVoltage() {
        rightMotor.setVoltage(IntakeConstants.ROLLER_VOLTAGE);
    }

    @Override
    public void extend() {
        extensionMotor.setVoltage(IntakeConstants.EXTEND_VOLTAGE);
    }

    @Override
    public Angle extensionPosition() {
        return extensionMotor.getPosition().getValue();
    }

    @Override
    public void retract() {
        extensionMotor.setVoltage(-IntakeConstants.EXTEND_VOLTAGE);
    }

    @Override
    public void close() throws Exception {
        rightMotor.close();
        extensionMotor.close();
    }

}
