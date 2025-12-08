package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase{

    private TalonFX m_motor = new TalonFX(ClimberConstants.kPort, "*");
    private double m_setPoint = getPosition();

    public Climber() {
        configureMotor();
    }

    public double getPosition() {
        return (m_motor.getPosition().getValueAsDouble());
    }

    public void setPosition(double position) {
        m_setPoint = position;
        m_motor.setControl(new MotionMagicVoltage(m_setPoint));
    }

    public void stop(){
        m_motor.stopMotor();
    }

    public Command setPositionCommand(double position) {
        return runOnce(() -> setPosition(position));
    }

    public Command setVoltage(double volts){
        return runOnce(()->m_motor.setVoltage(volts));
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    public boolean finishedMove(){
        return (Math.abs(m_motor.getPosition().getValueAsDouble() - m_setPoint)<= 5.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CLimber Position", getPosition());
    }

    private void configureMotor() {
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ClimberConstants.kStatorLimit)
                .withSupplyCurrentLimit(ClimberConstants.kSupplyLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

                m_motor.getStatorCurrent().setUpdateFrequency(50);
    
        m_motor.getConfigurator().apply(new Slot0Configs()
                .withKP(ClimberConstants.kP)
                .withKS(ClimberConstants.kS)
                .withKV(ClimberConstants.kV));
        
        m_motor.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.kAccel)
                .withMotionMagicCruiseVelocity(ClimberConstants.kVelocity));
    }
}