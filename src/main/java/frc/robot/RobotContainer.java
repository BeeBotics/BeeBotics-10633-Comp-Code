
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignWithLeftReefCommand;
import frc.robot.commands.AlignWithRightReefCommand;
import frc.robot.commands.AlignWithSourceCommand;
import frc.robot.commands.MoveArmToRotationCommand;
import frc.robot.commands.MoveElevatorToPositionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
        // The robot's subsystems
        double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("");
        
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final elevator m_elevator = new elevator();
        private final Arm m_arm = new Arm();
        private final AprilTagPID m_AprilTagPID = new AprilTagPID(m_robotDrive);
        private final SendableChooser<Command> autoChooser;

        // Controllers
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings

                NamedCommands.registerCommand("Place L4",
                                new MoveElevatorToPositionCommand(m_elevator, -0.95).alongWith(
                                                new MoveArmToRotationCommand(m_arm, -0.022)));
                NamedCommands.registerCommand("Down", new MoveArmToRotationCommand(m_arm, -0.008));
                NamedCommands.registerCommand("ReHome",
                                new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                                                new MoveArmToRotationCommand(m_arm, 0)));
                NamedCommands.registerCommand("Yoink",
                                new MoveElevatorToPositionCommand(m_elevator, 0.5).alongWith(
                                                new MoveArmToRotationCommand(m_arm, 0)));
                NamedCommands.registerCommand("Right Alignment", new AlignWithRightReefCommand(m_robotDrive, m_AprilTagPID));
                NamedCommands.registerCommand("Left Alignment", new AlignWithLeftReefCommand(m_robotDrive, m_AprilTagPID));
                NamedCommands.registerCommand("Source Alignment", new AlignWithSourceCommand(m_robotDrive, m_AprilTagPID));

                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(

                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                0.15),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                0.15),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                0.15),
                                                                true),
                                                m_robotDrive));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */

        // Driver Controller Buttons
        private void configureButtonBindings() {
                new JoystickButton(m_driverController, Button.kR1.value)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));
                new JoystickButton(m_driverController, Button.kL1.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));

                // Auto Alignment Buttons
                

                // new JoystickButton(m_driverController, Button.kCircle.value)
                //                 .whileTrue(new AlignWithLeftReefCommand(m_robotDrive, m_AprilTagPID));
                // new JoystickButton(m_driverController, Button.kCross.value)
                //                 .whileTrue(new AlignWithRightReefCommand(m_robotDrive, m_AprilTagPID));
                if (robotSpace != null) {
                new JoystickButton(m_driverController, Button.kCircle.value) // X button
                                .whileTrue(new AlignWithSourceCommand(m_robotDrive, m_AprilTagPID));
                }
        // Operator Controller Buttons
                // L4

                m_operatorController.x().whileTrue(
                        new MoveElevatorToPositionCommand(m_elevator, -0.95).alongWith(
                                        new MoveArmToRotationCommand(m_arm, -0.022)))
                        .whileFalse(new MoveArmToRotationCommand(m_arm, -0.013).andThen(new WaitCommand(0.5)).andThen(
                        (new MoveArmToRotationCommand(m_arm, 0).andThen(
                                new MoveElevatorToPositionCommand(m_elevator, 0)))));
                // L3
               
                m_operatorController.b().whileTrue(
                        new MoveElevatorToPositionCommand(m_elevator, -0.05).alongWith(
                                        new MoveArmToRotationCommand(m_arm, -0.022)))
                        .whileFalse(new MoveArmToRotationCommand(m_arm, -0.014).andThen(new WaitCommand(0.5)).andThen(
                        (new MoveArmToRotationCommand(m_arm, 0).andThen(
                                new MoveElevatorToPositionCommand(m_elevator, 0)))));
                // L2
               
                m_operatorController.y().whileTrue(
                        new MoveArmToRotationCommand(m_arm, -0.017).andThen(
                                                 new MoveElevatorToPositionCommand(m_elevator, 0.16)))
                        .whileFalse(new MoveArmToRotationCommand(m_arm, -0.013).andThen(new WaitCommand(0.5)).andThen(
                        (new MoveArmToRotationCommand(m_arm, 0).andThen(
                                new MoveElevatorToPositionCommand(m_elevator, 0)))));
                // L1

                m_operatorController.rightBumper().whileTrue(
                                new MoveArmToRotationCommand(m_arm, -0.016).andThen(
                                                new MoveElevatorToPositionCommand(m_elevator, 0.6)))
                                .whileFalse(new MoveElevatorToPositionCommand(m_elevator, 0).andThen(
                                                new MoveArmToRotationCommand(m_arm, 0)));
                // Yoink

                m_operatorController.a().whileTrue(
                                new MoveElevatorToPositionCommand(m_elevator, 0.6).alongWith(
                                                new MoveArmToRotationCommand(m_arm, 0)))
                                .whileFalse(
                                                new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                                                                new MoveArmToRotationCommand(m_arm, 0)));
                // Reset Arm

                m_operatorController.leftBumper().whileTrue(
                                new MoveArmToRotationCommand(m_arm, 0.005)).whileFalse(
                                                new InstantCommand(() -> m_arm.resetRotation()));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Returns the autonomous command
                return autoChooser.getSelected();
        }

        public void resetGyro(double angle) {
                m_robotDrive.setGyro(angle);
        }
}
