package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.SourceAlign;
import frc.robot.commands.SourceInsideAlign;
import frc.robot.constants.DriverConstants;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.constants.ShoulderConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turrent;
import frc.robot.Utilties.SmartShootByPose;
import frc.robot.subsystems.DangerZone;

public class RobotContainer {
        private final CommandXboxController m_driver = new CommandXboxController(DriverConstants.kPort);
        private final CommandXboxController m_operator = new CommandXboxController(DriverConstants.kTrey);
        private final DangerZone m_SSM = new DangerZone(m_climber, m_shoulder, m_manipulator, m_elevator, m_turrent, m_smartShoot);
        private final Manipulator m_manipulator = new Manipulator();
        private final Turrent m_turrent = new Turrent();
        private final Shoulder m_shoulder = new Shoulder();
        private final Elevator m_elevator = new Elevator();
        private final Climber m_climber = new Climber();
        private final SmartShootByPose m_smartShoot = new SmartShootByPose(m_turrent, m_manipulator);
        public final CommandSwerveDrivetrain m_drivetrain;

        public RobotContainer() {
                m_drivetrain = TunerConstants.createDrivetrain();
                configureNamedCommands();
                autoChooser = AutoBuilder.buildAutoChooser("Center");
                SmartDashboard.putData("Auto Mode", autoChooser);
                configureDefaultCommands();
                ConfigureBindings();
        }

        private final SendableChooser<Command> autoChooser;

        private void configureDefaultCommands() {
            m_drivetrain.setDefaultCommand(DriveCommands.fieldOrientedDrive(m_drivetrain,
                                                () -> m_driver.getLeftY(),
                                                () -> m_driver.getLeftX(),
                                                () -> m_driver.getRightX()));
        }

        

        private void ConfigureBindings() {
                m_driver.leftTrigger().whileTrue(m_SSM.manage(ManipulatorConstants.kIntakeVoltage, SubsystemID.Manipulator));
                m_driver.rightTrigger().whileTrue(m_smartShoot.smartShoot());

                m_driver.rightTrigger().and().rightBumper()
                                        .whileTrue(m_SSM.manage(ManipulatorConstants.kShootVoltage, SubsystemID.Manipulator));
                
                m_operator.leftTrigger().whileTrue(m_SSM.manage(1, SubsystemID.Elevator));
                m_operator.rightTrigger().whileTrue(m_SSM.manage(1, SubsystemID.Shoulder))
                                                .onFalse(m_SSM.manage(ShoulderConstants.kTrough, SubsystemID.Shoulder));

                m_operator.a().whileTrue(m_SSM.manage(ClimberConstants.kClimb, SubsystemID.Climber));

                m_operator.povUp().onTrue(new InstantCommand(()-> m_elevator.poseAdjust(5)));
                m_operator.povDown().onTrue(new InstantCommand(()-> m_elevator.poseAdjust(5)));

                m_operator.povLeft().onTrue(new InstantCommand(()-> m_turrent.AngleAdjust(5)));
                m_operator.povRight().onTrue(new InstantCommand(()-> m_turrent.AngleAdjust(-5)));
        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
}
