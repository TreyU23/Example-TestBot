package.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystem.DangerZone;

public class Turret extends SubsystemBase {
    private TalonFX m_motor = new TalonFX(TurretConstants.kPort, "*");
    private double m_setPoint = getPosition();
    private DangerZone m_SSM;

    public Turret() {
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
        m_SSM.manage((getPosition() + adjust), SubsystemID.Turret);
    }

    public double getSmartPose() {
        return m_smartPose;
    }

    private void motorConfigs() {
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(TurretConstants.kStatorLimit)
        .withSupplyCurrentLimit(TurretConstants.kSupplyLimit)
        .withSupplyCurrentLimitEnable(true));

        m_motor.getConfigurator().apply(new Slot0Configs()
        .withKP(TurretConstants.kP)
        .withKI(TurretConstants.kI)
        .withKD(TurretConstants.kD)
        .withKG(TurretConstants.kG)
        .withKS(TurretConstants.kS)
        .withKV(TurretConstants.kV)
        .withKA(TurretConstants.kA)
        .withGravityType(GravityTypeValue.Arm_Cosine));

        m_motor.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(TurretConstants.kAccel)
        .withMotionMagicCruiseVelocity(TurretConstants.kVelocity));

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position", getPosition());
        SmartDashboard.putNumber("Turret Set Point", m_setPoint);
        SmartDashboard.putNumber("Turret Smart Pose", m_smartPose);
    }
}