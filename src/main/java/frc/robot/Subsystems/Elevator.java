package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    public final static TalonFX m_rightMotor = new TalonFX(11, "*");
    public final static TalonFX m_leftMotor = new TalonFX(12, "*");
    public double m_setPoint = getPosition();
    private SoftwareLimitSwitchConfigs m_limits;

    public Elevator() {
        configureLeftRightMotors();
    }

    public void setPosition(double position){
        m_setPoint = position;
        if (position > getPosition()) {
            m_rightMotor.setControl(new DynamicMotionMagicVoltage(position/ElevatorConstants.kInchPerRotation, 
                ElevatorConstants.kUpVelocityElevator, ElevatorConstants.kUpAccelerationElevator, ElevatorConstants.kUpJerkElevator));
        } else {
            m_rightMotor.setControl(new DynamicMotionMagicVoltage(position/ElevatorConstants.kInchPerRotation, 
                ElevatorConstants.kDownVelocityElevator, ElevatorConstants.kDownAccelerationElevator, ElevatorConstants.kDownJerkElevator));
        }
    }

    public void poseAdjust(double adjust) {
        setPosition(getPosition() + adjust);
    }

    @SuppressWarnings("rawtypes")
    public StatusSignal getCurrentLeft() {
        return (m_leftMotor.getSupplyCurrent());
    }

    @SuppressWarnings("rawtypes")
    public StatusSignal getCurrentRight() {
        return (m_rightMotor.getSupplyCurrent());
    }

    public double getPosition() {
        return (ElevatorConstants.kInchPerRotation*m_rightMotor.getPosition().getValueAsDouble());
    }

    public void enableLimits(boolean limitsState){
        m_limits.ReverseSoftLimitEnable = limitsState;
        m_limits.ForwardSoftLimitEnable = limitsState;
        m_rightMotor.getConfigurator().apply(m_limits);
    } 
    
    public void stop() {
        m_rightMotor.set(0);
    }

    public boolean atSetpoint(){
        return Math.abs(getPosition()-m_setPoint) <= 1.0;
    }

    public void jogging(boolean direction){
        if (direction == true){
            SmartDashboard.putNumber("Elevator Position", getPosition());
            setPosition(getPosition()+2.0*ElevatorConstants.kInchPerRotation);
        }
        else{
            SmartDashboard.putNumber("Elevator Position", getPosition());
            setPosition(getPosition()-2.0*ElevatorConstants.kInchPerRotation);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Left Elevator Current", getCurrentLeft().getValueAsDouble());
        SmartDashboard.putNumber("Right Elevator Current", getCurrentRight().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Set Point", m_setPoint);
    }

    private void configureLeftRightMotors() {
        m_leftMotor.setControl(new Follower(9, true).withUpdateFreqHz(250));

        m_rightMotor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ElevatorConstants.ElevatorCurrents.kStatorCurrent)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ElevatorConstants.ElevatorCurrents.kSupplyCurrent)
            .withSupplyCurrentLimitEnable(true));

        m_leftMotor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ElevatorConstants.ElevatorCurrents.kStatorCurrent)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ElevatorConstants.ElevatorCurrents.kSupplyCurrent)
            .withSupplyCurrentLimitEnable(true));
            
         m_rightMotor.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicAcceleration(ElevatorConstants.kDownAccelerationElevator)
            .withMotionMagicCruiseVelocity(ElevatorConstants.kDownVelocityElevator)
            .withMotionMagicJerk(ElevatorConstants.kDownJerkElevator));
                    
        m_rightMotor.getConfigurator().apply( new Slot0Configs()
            .withKP(ElevatorConstants.kPElevator)
            .withKI(ElevatorConstants.kIElevator)
            .withKD(ElevatorConstants.kDElevator)
            .withKG(ElevatorConstants.kGElevator)
            .withKS(ElevatorConstants.kSElevator)
            .withKV(ElevatorConstants.kVElevator)
            .withKA(ElevatorConstants.kAElevator)
            .withGravityType(GravityTypeValue.Elevator_Static));
                        
        m_rightMotor.getConfigurator().apply(m_limits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.kUpperLimitElevator/ElevatorConstants.kInchPerRotation)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.kULowerLimitElevator/ElevatorConstants.kInchPerRotation));

        m_rightMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));                       
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
    }
}