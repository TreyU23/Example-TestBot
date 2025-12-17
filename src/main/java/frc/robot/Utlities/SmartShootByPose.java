package frc.robot.utlities;

import frc.robot.subsystems.Turrent;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SmartShootByPose {
        //Creates the objects and values needed throught the subsystem.
    private Turrent m_turrent;
    private Manipulator m_manipulator;
    private double currentRotation = 0.0;
    private double MAX_VALUE = 0.0;


        //Defines the target rotations for shooting.
    private emum Rotation2d[] TargetRot = {
        new Rotation2d(0.0),
        //Pose 1
        new Rotation2d(0.0)
        //Pose 2
    };

    @Override
    public void periodic() {
        currentRotation = m_turrent.getRotation();
    }

        //Calculates the closest target rotation to the current rotation.
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


        //Declares the constructors for the SmartShootByPose subsystem.
    public SmartShootByPose(Turrent turrent, Manipulator manipulator) {
        m_turrent = turrent;
        m_manipulator = manipulator;
    }

        //Returns the adjusted angle for the turrent to shoot at.
    private double AngleAdjust(double currentAngle) {
        double AngleFromGoal = currentAngle - calculate(currentAngle);
        return (AngleFromGoal + currentAngle);
        SmartDashboard.putNumber("Adjusted Angle", AngleFromGoal + currentAngle);
    }


        //Calculates the time needed for the turrent to rotate to the target angle.
    private waitCalc(double targetAngle) {
        double distance = Math.abs(targetAngle - currentRotation);
        double time = distance / Turrent.kVelocity;
        return ((time*1.10) * 1000);
        SmartDashboard.putNumber("Smart Shoot Wait Time", ((time*1.10) * 1000));
    }


        //Smart shoot command that rotates the turrent to the target angle and shoots.
    public Command smartShoot() {
        return runOnce(() -> m_turrent.setAngle(AngleAdjust(currentRotation))
                                //Sets the turrent to the adjusted angle.
            .andThen(() -> new WaitCommand(waitCalc(AngleAdjust(currentRotation)))
                                //Waits for the turrent to reach the target angle.
            .andThen(() -> m_manipulatior.setVoltage(ManipulatorConstants.kShootVoltage))));
                                //Sets the manipulator to shoot.
    }
}