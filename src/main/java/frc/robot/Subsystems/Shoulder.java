package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {

    private final TalonFX m_rightMotor = new TalonFX(ShoulderConstants.kRightID, "*");
    private final TalonFX m_leftMotor = new TalonFX(ShoulderConstants.kLeftID, "*");
    private final Climber m_climber = new Climber();

    private double m_setPoint = getPosition();
    private CANcoder m_cancoder = new CANcoder(25, "*");
    private double dangerZone = 50;

    private double climberPose = m_climber.getPosition();

    public Shoulder() {
        configureCancoder();
        configureRightMotor();
        configureLeftMotor();
    }

    
    public double getPosition() {
        return (m_rightMotor.getPosition().getValueAsDouble());
    }

    public long waitCalc(double targetPosition) {
        double distance = Math.abs(targetPosition - m_climber.getPosition());
        double time = distance / ClimberConstants.kVelocity;
        return (long) ((time*1.10) * 1000);
    }

    public void setPosition(double position) {
        m_setPoint = position;
        if (m_setPoint > ShoulderConstants.kUpLimit) {
            m_setPoint = ShoulderConstants.kUpLimit;
        } else if (m_setPoint < ShoulderConstants.kLowerLimit) {
            m_setPoint = ShoulderConstants.kLowerLimit;
        }

        if (climberPose == dangerZone) {
            m_climber.setPosition(ClimberConstants.kSafeHeight);
            try {
                wait(waitCalc(m_setPoint));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            m_rightMotor.setControl(new MotionMagicVoltage(m_setPoint));
        }
        m_rightMotor.setControl(new MotionMagicVoltage(m_setPoint));
    }

    public void stop() {
        m_rightMotor.stopMotor();
    }

    public double getCurrentLeft() {
        return (m_leftMotor.getSupplyCurrent().getValueAsDouble());
    }

    public double getCurrentRight() {
        return (m_rightMotor.getSupplyCurrent().getValueAsDouble());
    }

    public Command climb(){
        return runOnce(()->m_rightMotor.setVoltage(-2.0));
    }

    public Command setPositionCommand(double position) {
        return runOnce(() -> setPosition(position));
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Position", getPosition());
        SmartDashboard.putNumber("Left Shoulder Current", getCurrentLeft());
        SmartDashboard.putNumber("Right Shoulder Current", getCurrentRight());
        SmartDashboard.putNumber("Shoulder Set Point", m_setPoint);
    }

    public void configureCancoder() {
        m_cancoder.getConfigurator().apply(new MagnetSensorConfigs()
                .withMagnetOffset(ShoulderConstants.kMangentOffset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    public void configureRightMotor() {
        m_rightMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ShoulderConstants.kStatorLimit)
                .withSupplyCurrentLimit(ShoulderConstants.kSupplyLimit)
                .withSupplyCurrentLimitEnable(true));

        m_rightMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(ShoulderConstants.kP)
                .withKI(ShoulderConstants.kI)
                .withKD(ShoulderConstants.kD)
                .withKG(ShoulderConstants.kG)
                .withKS(ShoulderConstants.kS)
                .withKV(ShoulderConstants.kV)
                .withKA(ShoulderConstants.kA)
                .withGravityType(GravityTypeValue.Arm_Cosine));

        m_rightMotor.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ShoulderConstants.kAccel)
                .withMotionMagicCruiseVelocity(ShoulderConstants.kVelocity));

        m_rightMotor.getConfigurator().apply(new FeedbackConfigs()
                .withRemoteCANcoder(m_cancoder));

        m_rightMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShoulderConstants.kUpLimit)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShoulderConstants.kLowerLimit));

        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void configureLeftMotor() {
        m_leftMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ShoulderConstants.kStatorLimit)
                .withSupplyCurrentLimit(ShoulderConstants.kSupplyLimit)
                .withSupplyCurrentLimitEnable(true));
        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leftMotor.setControl(new Follower(ShoulderConstants.kRightID, true));
    }
}