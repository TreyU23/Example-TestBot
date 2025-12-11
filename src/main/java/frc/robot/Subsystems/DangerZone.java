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

    public DangerZone(Climber climber, Shoulder shoulder, Manipulator manipulator,
                  Elevator elevator, Turrent turrent, SmartShootByPose smartShoot) {
        m_climber = climber;
        m_shoulder = shoulder;
        m_manipulator = manipulator;
        m_elevator = elevator;
        m_turrent = turrent;
        m_smartShoot = smartShoot;
    }

    public emum SubsystemID {
        Shoulder,
        Climber,
        Manipulator,
        Elevator,
        Turrent
    }

    private final double getPostiion(SubsystemID subsystem) {
        if (subsystem == SubsystemID.Shoudler) {
            return m_shoulder.getPosition();
        } else if (subsystem == SubsystemID.Climber) {
            return m_climber.getPosition();
        } else if (subsystem == SubsystemID.Elevator){
            return m_elevator.getPosition();
        } else if (subsystem == SubsystemID.Turrent) {
            return m_turrent.getPosition();
        }
    }

    private final double getVelocity(SubsystemID subsystem) {
        if (subsystem == SubsystemID.Shoudlder) {
            return m_shoulder.getVelocity();
        } else if (subsystem == SubsystemID.Climber) {
            return m_climber.getVelocity();
        } else if (subsystem == SubsystemID.Elevator){
            return m_elevator.getVelocity();
        } else if (subsystem == SubsystemID.Turrent) {
            return m_turrent.getVelocity();
        } else if (subsystem == SubsystemID.Manipulator) {
            return m_manipulator.getVelocity();
        }
    }

    private double waitCalc(double targetPosition,SubsystemID subsystem) {
        double distance = Math.abs(targetPosition - getPosition(subsystem));
        double time = distance / getVelocity(subsystem);
        return (time*0.1);
    }
        
    public Command manage(double input,SubsystemID subsystem) {
        if (subsystem == SubsystemID.Shoulder) {
            if (Math.abs(input) <= ShoulderConstants.kDanger 
                || Math.abs(input) >= ShoulderConstants.kDangerLow) {
                    return m_climber.setPosition(ClimberConstants.kSafeHeight)
                        .alongWith(() -> {
                            new WaitCommand(waitCalc(ClimberConstants.kSafeHeight, Climber)))}
                    .andThen(() -> {
                        m_shoulder.setPosition(input);
                    });

            } else {//if in safe spot dirrect input.
                return m_shoulder.setPosition(input);
            };

        } else if (subsystem == SubsystemID.Climber) {
            if (Math.abs(input) <= ClimberConstants.kDanger) {
                    return m_shoulder.setPosition(ShoulderConstants.kSafeHeight)
                        .alongWith(() -> {
                            new WaitCommand(waitCalc(ShoulderConstants.kSafeHeight, Shoulder)))}
                    .andThen(() -> {
                        m_climber.setPosition(input);
                    });

            } else {//if in safe spot dirrect input.
                return m_climber.setPosition(input);
            };

        } else if (subsystem == SubsystemID.Manipulator){
            return m_manipulator.setVoltage(input);

        } else if (subsystem == SubsystemID.Elevator){
            return m_elevator.setPosition(input);

        } else if (subsystem == SubsystemID.turrent){
            return m_smartShoot.smartShoot();
        }
    };
}