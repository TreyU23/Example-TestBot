package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;

public class manipulator extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(ManipulatorConstants.kID, "rio");
        //Creates a motor controller object for the manipulator
    private final Debouncer m_currentDebounce = new Debouncer(0.090, DebounceType.kBoth);
    private double m_voltOutput = 0.0;

    public manipulator() {
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ManipulatorConstants.kStatorLimit)
                .withSupplyCurrentLimit(ManipulatorConstants.kSupplyLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        m_motor.getStatorCurrent().setUpdateFrequency(50);
    }

    public void setVoltage(double voltage) {
        m_voltOutput = voltage;
        m_motor.setVoltage(voltage);
    }

    public void stop() {
        m_voltOutput = 0.0;
        m_motor.stopMotor();
    }

    public boolean hasCoral(){
        return m_currentDebounce.calculate(m_motor.getStatorCurrent().getValueAsDouble() >= 80.0 && m_voltOutput < -0.1);
    }

    public Command holdCmd(){
        return runOnce(()-> setVoltage(-0.5));
    }

    public Command setVoltageCmd(double voltage) {
        return runOnce(() -> setVoltage(voltage));
    }

    public Command stopCmd(){
        return runOnce(()->stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Manipulator Current", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Manipulator Temp(C)", m_motor.getDeviceTemp().getValueAsDouble());
    }
}
