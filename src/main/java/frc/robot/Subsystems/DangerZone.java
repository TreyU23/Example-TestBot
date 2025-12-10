package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ShoulderConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turrent;
import frc.robot.Utilties.SmartShootByPose;

public class DangerZone extends SubsystemBase {
    private final Climber m_climber;
    private final Shoulder m_shoulder;
    private final Manipulator m_manipulator;
    private final Elevator m_elevator;
    private final Turrent m_turrent;
    private final SmartShootByPose m_smartShoot;

    private final double getPostiion(subsystem) {
        if (subsystem.equals(Shoulder)) {
            return m_shoulder.getPosition();
        } else if (subsystem.equals(Climber)) {
            return m_climber.getPosition();
        } else if (subsystem.equals(Elevator)){
            return m_elevator.getPosition();
        }
    }

    private final double getVelocity(subsystem) {
        if (subsystem.equals(Shoulder)) {
            return m_shoulder.getVelocity();
        } else if (subsystem.equals(Climber)) {
            return m_climber.getVelocity();
        } else if (subsystem.equals(Elevator)){
            return m_elevator.getVelocity();
        }
    }

    private long waitCalc(double targetPosition, subsystem) {
        double distance = Math.abs(targetPosition - getPosition(subsystem));
        double time = distance / getVelocity();
        return (long) ((time*1.10) * 1000);
    }

    public DangerZone (
        
    public void manage(double input, subsystem) {
        if (subsystem.equals(Shoulder)) {
            if (Math.abs(input) <= ShoulderConstants.kDanger 
                or Math.abs(input) >= ShoulderConstants.kDangerLow) {
                    m_climber.setPosition(ClimberConstants.kSafeHeight)
                        .alongWith(() -> {
                    try {
                        wait(waitCalc(input, m_climber));
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }}).alongWith(() -> {
                        m_shoulder.setPosition(input);
                    });

            } else {//if in safe spot dirrect input.
                m_shoulder.setPosition(input);
            };

        } else if (subsystem.equals(Climber)) {
            if (Math.abs(input) <= ClimberConstants.kDanger) {
                    m_shoulder.setPosition(ShoulderConstants.kSafeHeight)
                        .alongWith(() -> {
                    try {
                        wait(waitCalc(input, m_climber));
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }}).alongWith(() -> {
                        m_climber.setPosition(input);
                    });

            } else {//if in safe spot dirrect input.
                m_climber.setPosition(input);
            };

        } else if (subsystem.equals(Manipulator)){
            m_manipulatior.setVoltage(input);

        } else if (subsystem.equals(Elevator)){
            m_elevator.setPosition(input);

        } else if (subsystem.equals(turrent)){
            m_smartShoot.smartShoot();
        }
    });
}

