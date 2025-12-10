package.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurrentConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystem.DangerZone;

public class Turrent extends SubsystemBase {
    private TalonFX m_motor = new TalonFX(TurrentConstants.kPort, "*");
    private double m_setPoint = getPosition();
    private DangerZone m_SSM;

    public Turrent() {
        motorConfigs();
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public void setPosition(double position) {
        m_setPoint = position;
        m_motor.setControl(new MotionMagicVoltage(m_setPoint));
    }

    public void poseAdjust(double adjust) {
        m_SSM.if((getPosition() + adjust), Turrent);
    }

    public double getSmartPose() {
        return m_smartPose;
    }

    private void motorConfigs() {
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(TurrentConstants.kStatorLimit)
        .withSupplyCurrentLimit(TurrentConstants.kSupplyLimit)
        .withSupplyCurrentLimitEnable(true));

        m_motor.getConfigurator().apply(new Slot0Configs()
        .withKP(TurrentConstants.kP)
        .withKI(TurrentConstants.kI)
        .withKD(TurrentConstants.kD)
        .withKG(TurrentConstants.kG)
        .withKS(TurrentConstants.kS)
        .withKV(TurrentConstants.kV)
        .withKA(TurrentConstants.kA)
        .withGravityType(GravityTypeValue.Arm_Cosine));

        m_motor.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(TurrentConstants.kAccel)
        .withMotionMagicCruiseVelocity(TurrentConstants.kVelocity));

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turrent Position", getPosition());
        SmartDashboard.putNumber("Turrent Set Point", m_setPoint);
        SmartDashboard.putNumber("Turrent Smart Pose", m_smartPose);
    }
}