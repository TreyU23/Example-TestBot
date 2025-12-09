package frc.robot.utlities;

import frc.robot.subsystems.Turrent;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SmartShootByPose {
    private Turrent m_turrent;
    private Manipulator m_manipulator;
    private double currentRotation = 0.0;
    private double MAX_VALUE = 0.0;

    private emum Rotation2d[] TargetRot = {
        new Rotation2d(0.0),
        new Rotation2d(0.0)
    };

    @Override
    public void periodic() {
        currentRotation = m_turrent.getRotation();
    }

    private static Rotation2d calculate(Translation2d currentRotation) {
        double cloesstDistance = Double.MAX_VALUE;
        Rotation2d closestAngle = new Rotation2d();
        for (Rotation2d rot : TargetRot) {
            double distance = currentRotation.getDistance(rot.getTranslation());
            if (distance <= closestAngle) {
                closestDistance = distance;
                closestAngle = rot;
            }
        }
        return closestAngle;
        SmartDashboard.putNumber("Closest Angle", closestAngle.getDegrees());
    }

    public SmartShootByPose(Turrent turrent, Manipulator manipulator) {
        m_turrent = turrent;
        m_manipulator = manipulator;
    }

    private double AngleAdjust(double currentAngle) {
        double AngleFromGoal = currentAngle - calculate(currentAngle);
        return (AngleFromGoal + currentAngle);
        SmartDashboard.putNumber("Adjusted Angle", AngleFromGoal + currentAngle);
    }

    private waitCalc(double targetAngle) {
        double distance = Math.abs(targetAngle - currentRotation);
        double time = distance / Turrent.kVelocity;
        return (long) ((time*1.10) * 1000);
        SmartDashboard.putNumber("Smart Shoot Wait Time", (long) ((time*1.10) * 1000));
    }

    public Command smartShoot() {
        return runOnce(() -> m_turrent.setAngle(AngleAdjust(currentRotation))
            .alongWith(() -> wait(waitCalc(AngleAdjust(currentRotation))
            .alongWith(() -> m_manipulatior.setVoltage(ManipulatorConstants.kShootVoltage)))));
    }
}