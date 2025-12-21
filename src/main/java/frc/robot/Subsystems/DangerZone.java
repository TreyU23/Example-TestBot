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
import frc.robot.subsystems.Turret;
import frc.robot.Utilties.SmartShootByPose;

public class DangerZone extends SubsystemBase {

        //Declare subsystems in constructor.
    public DangerZone(Climber climber, Shoulder shoulder, Manipulator manipulator,
                  Elevator elevator, Turret turret, SmartShootByPose smartShoot) {
        m_climber = climber;
        m_shoulder = shoulder;
        m_manipulator = manipulator;
        m_elevator = elevator;
        m_turret = turret;
        m_smartShoot = smartShoot;
    }


        //Subsystem identifiers.
    public emum SubsystemID {
        Shoulder,
        Climber,
        Manipulator,
        Elevator,
        Turret
    }

    private var State = notInUse;
    private var nextState = notInUse;

        //Helper methods to get position and velocity of subsystems.
    private final double getPostiion(SubsystemID subsystem) {
        if (subsystem == SubsystemID.Shoudler) {
            return m_shoulder.getPosition();
        } else if (subsystem == SubsystemID.Climber) {
            return m_climber.getPosition();
        } else if (subsystem == SubsystemID.Elevator){
            return m_elevator.getPosition();
        } else if (subsystem == SubsystemID.Turret) {
            return m_turret.getPosition();
        }
    }

    private final double getVelocity(SubsystemID subsystem) {
        if (subsystem == SubsystemID.Shoudlder) {
            return m_shoulder.getVelocity();
        } else if (subsystem == SubsystemID.Climber) {
            return m_climber.getVelocity();
        } else if (subsystem == SubsystemID.Elevator){
            return m_elevator.getVelocity();
        } else if (subsystem == SubsystemID.Turret) {
            return m_turret.getVelocity();
        } else if (subsystem == SubsystemID.Manipulator) {
            return m_manipulator.getVelocity();
        }
    }


        //Calculates needed time to wait till at the safe pose.
    private double waitCalc(double targetPosition, SubsystemID subsystem) {
        double distance = Math.abs(targetPosition - getPosition(subsystem));
        double time = distance / getVelocity(subsystem);
        return (time*0.1);
    }
       
    
        //Main method to manage dangerous movements.
    public Command manage(double input, SubsystemID subsystem) {
        if (subsystem == SubsystemID.Shoulder) {
            if ((Math.abs(input) <= ShoulderConstants.kDanger)
                || (Math.abs(input) >= ShoulderConstants.kDangerLow)) 
                && ((getPostiion(Climber) <= ClimberConstants.kDanger) 
                || (getPostiion(Climber) >= ClimberConstants.kDangerLow)) {
                    return m_climber.setPosition(ClimberConstants.kSafeHeight)
                        .alongWith(() -> new InstantCommand(State = Climber))
                        .alongWith(() -> new InstantCommand(nextState = Shoulder))
                        .alongWith(() -> 
                            new WaitCommand(waitCalc(ClimberConstants.kSafeHeight, Climber)))
                    .andThen(() -> 
                        m_shoulder.setPosition(input)
                        .alongWith(() -> new InstantCommand(State = Shoulder))
                        .alongWith(() -> new InstantCommand(nextState = notInUse))
                    );

            } else {//if in safe spot dirrect input.
                return m_shoulder.setPosition(input)
                    .alongWith(() -> new InstantCommand(State = Shoulder));
            };

        } else if (subsystem == SubsystemID.Climber) {
            if (((Math.abs(input) <= ClimberConstants.kDanger)
                || (Math.abs(input) >= ClimberConstants.kDangerLow))
                && ((getPostiion(Shoulder) <= ShoulderConstants.kDanger)
                || (getPostiion(Shoulder) >= ShoulderConstants.kDangerLow))) {
                    return m_shoulder.setPosition(ShoulderConstants.kSafeHeight)
                        .alongWith(() -> new InstantCommand(State = Shoulder))
                        .alongWith(() -> new InstantCommand(nextState = Climber))
                        .alongWith(() -> 
                            new WaitCommand(waitCalc(ShoulderConstants.kSafeHeight, Shoulder)))
                    .andThen(() -> 
                        m_climber.setPosition(input)
                        .alongWith(() -> new InstantCommand(State = Climber))
                        .alongWith(() -> new InstantCommand(nextState = notInUse))
                    );

            } else {
                return m_climber.setPosition(input)
                .alongWith(() -> new InstantCommand(State = Climber));
            };

        } else if (subsystem == SubsystemID.Manipulator){
            return m_manipulator.setVoltage(input)
                .alongWith(() -> new InstantCommand(State = Manipulator));

        } else if (subsystem == SubsystemID.Elevator){
            return m_elevator.setPosition(input)
                .alongWith(() -> new InstantCommand(State = Elevator));

        } else if (subsystem == SubsystemID.turret){
            return m_smartShoot.smartShoot()
                .alongWith(() -> new InstantCommand(State = Turret));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("SSM State", State);
        SmartDashboard.putData("SSM Next State", nextState);
    }
}