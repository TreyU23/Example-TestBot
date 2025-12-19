package frc.robot.utlities;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SmartShootByPose {
        //Creates the objects and values needed throught the subsystem.
    private Turret m_turret;
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

    /* Another Way of setting the target rotations.
            This will allow for you to declear the 
                values anywhere in the program.
    private Targets[] = new Rotation2d[5];
        Targets[1] = (0.0);
        Targets[2] = (0.0);
        Targets[3] = (0.0);
        Targets[4] = (0.0);
        Targets[5] = (0.0);
    */


    @Override
    public void periodic() {
        currentRotation = m_turret.getRotation();
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
    public SmartShootByPose(Turret turret, Manipulator manipulator) {
        m_turret = turret;
        m_manipulator = manipulator;
    }

        //Returns the adjusted angle for the turret to shoot at.
    private double AngleAdjust(double currentAngle) {
        double AngleFromGoal = currentAngle - calculate(currentAngle);
        return (AngleFromGoal + currentAngle);
        SmartDashboard.putNumber("Adjusted Angle", AngleFromGoal + currentAngle);
    }


        //Calculates the time needed for the turret to rotate to the target angle.
    private waitCalc(double targetAngle) {
        double distance = Math.abs(targetAngle - currentRotation);
        double time = distance / Turret.kVelocity;
        return ((time*1.10) * 1000);
        SmartDashboard.putNumber("Smart Shoot Wait Time", ((time*1.10) * 1000));
    }


        //Smart shoot command that rotates the turret to the target angle and shoots.
    public Command smartShoot() {
        return runOnce(() -> m_turret.setPosition(AngleAdjust(currentRotation))
                                //Sets the turret to the adjusted angle.
            .andThen(() -> new WaitCommand(waitCalc(AngleAdjust(currentRotation)))
                                //Waits for the turret to reach the target angle.
            .andThen(() -> m_manipulatior.setVoltage(ManipulatorConstants.kShootVoltage))));
                                //Sets the manipulator to shoot.
    }
}