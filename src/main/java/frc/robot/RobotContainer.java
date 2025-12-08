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
import frc.robot.subsystems.manipulator;

public class RobotContainer {
        private final CommandXboxController m_driver = new CommandXboxController(DriverConstants.kPort);
        private final CommandXboxController m_operator = new CommandXboxController(DriverConstants.kTrey);
        private final Shoulder m_shoulder = new Shoulder();
        private final Manipulator m_manipulator = new manipulator();
        private final Climber m_climber = new Climber();
        private final Elevator m_elevator = new Elevator();
        public final CommandSwerveDrivetrain m_drivetrain;


        private boolean readyToClimb = false;
        private boolean isArmOut = true;

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
                m_driver.povUp().onTrue(DriveCommands.resetFieldOrientation(m_drivetrain));

                m_driver.x().whileTrue(new SourceAlign(m_drivetrain,m_driver::getLeftY,m_driver::getLeftX)
                                        .alongWith(m_manipulator.setVoltageCmd(ManipulatorConstants.kIntakeVoltage)
                                                .alongWith(m_shoulder.setPositionCommand(ShoulderConstants.kLoadingStation))))
                                                        .onFalse(m_shoulder.setPositionCommand(ShoulderConstants.kTrough)
                                                                .alongWith(m_manipulator.setVoltageCmd(ManipulatorConstants.kHoldVoltage)));

                m_driver.b().whileTrue(new SourceInsideAlign(m_drivetrain,m_driver::getLeftY,m_driver::getLeftX)
                                        .alongWith(m_manipulator.setVoltageCmd(ManipulatorConstants.kIntakeVoltage)
                                                .alongWith(m_shoulder.setPositionCommand(ShoulderConstants.kLoadingStation))))
                                                        .onFalse(m_shoulder.setPositionCommand(ShoulderConstants.kTrough)
                                                                .alongWith(m_manipulator.setVoltageCmd(ManipulatorConstants.kHoldVoltage)));
                                                                

                m_driver.a().onTrue(m_climber.setPositionCommand(ClimberConstants.kClimbDown));

                m_operator.leftTrigger().or(m_operator.rightTrigger()).whileTrue(new ReefAlign(m_drivetrain,m_driver::getLeftY,m_driver::getLeftX, ()->
                        m_operator.rightTrigger().getAsBoolean(),()->m_operator.leftTrigger().getAsBoolean()));

                m_driver.leftTrigger().onTrue(m_shoulder.setPositionCommand(ShoulderConstants.kTroughtBump))
                                .onFalse(m_shoulder.setPositionCommand(ShoulderConstants.kTrough));

                m_driver.rightTrigger()
                                .onTrue(new ConditionalCommand(
                                                m_manipulator.setVoltageCmd(ManipulatorConstants.kScoreVoltageBump),
                                                m_manipulator.setVoltageCmd(ManipulatorConstants.kScoreVoltage),
                                                m_driver.leftTrigger()::getAsBoolean))
                                .onFalse(m_manipulator.stopCmd());

                m_driver.leftBumper().onTrue(m_manipulator.setVoltageCmd(ManipulatorConstants.kIntakeVoltage)
                                .alongWith(m_shoulder.setPositionCommand(ShoulderConstants.kFloorIntake))).onFalse(
                                                m_shoulder.setPositionCommand(ShoulderConstants.kTrough)
                                                                .alongWith(m_manipulator.setVoltageCmd(
                                                                                ManipulatorConstants.kHoldVoltage)));

                m_driver.rightBumper().onTrue(m_manipulator.setVoltageCmd(ManipulatorConstants.kIntakeVoltage)
                                .alongWith(m_shoulder.setPositionCommand(ShoulderConstants.kLoadingStation))).onFalse(
                                                m_shoulder.setPositionCommand(ShoulderConstants.kTrough)
                                                                .alongWith(m_manipulator.setVoltageCmd(
                                                                                ManipulatorConstants.kHoldVoltage)));

                m_driver.back()
                            .onTrue(m_shoulder.setPositionCommand(ShoulderConstants.kClimbStart)
                                            .alongWith(m_climber.setPositionCommand(ClimberConstants.kPrepClimb))
                                            .alongWith(new InstantCommand(() -> readyToClimb = true))
                                            .alongWith(new InstantCommand(() -> isArmOut = false)));
                m_driver.start()
                            .onTrue(new ConditionalCommand(
                                            m_climber.setPositionCommand(ClimberConstants.kClimb)
                                                            .andThen(new WaitUntilCommand(m_climber::finishedMove))
                                                            .andThen(m_climber.setVoltage(-1.25)),
                                                            new WaitCommand(0.0), () -> readyToClimb));
                m_operator.rightBumper()
                            .whileTrue(m_elevator.setPositionCommand(24.0));
                
                m_operator.leftBumper()
                            .onTrue(m_elevator.setPositionCommand(0.0));
                
                m_operator.povUp()
                            .onTrue(m.elevator.poseAdjust(5.0));
                m_operator.povDown()
                            .onTrue(m.elevator.poseAdjust(-5.0));

        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
}
