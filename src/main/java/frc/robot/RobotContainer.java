package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.JoystickF310.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TeleopArcadeDriveCommand;

import static frc.robot.Constants.*;

public class RobotContainer {
    // The robot's subsystems and commands are defined in this file

    //Create joystick input objects 
    private final JoystickF310 joystickDrive = new JoystickF310(Ports.PORT_JOYSTICK_DRIVE);
    private final JoystickF310 joystickOperator = new JoystickF310(Ports.PORT_JOYSTICK_OPERATOR);

    // Create subsystem objects
    private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    //Create command objects
    private final AutoCommand m_autoCommand = new AutoCommand(m_drivetrain);
    private final TeleopArcadeDriveCommand m_driveCommand = new TeleopArcadeDriveCommand(m_drivetrain);
    private final ArmManualCommand m_armManualCommand = new ArmManualCommand(m_arm);
    private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intake);

    /** The container for the robot. Contains subsystems, OI devices, and commands. **/
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();

    }

    private void configureControlBindings() {
        // Set suppliers for commands with joystick axes:
        m_driveCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_THROTTLE_EXPONENT),
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_TURNING_EXPONENT)
        );
        
        m_armManualCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_THROTTLE_EXPONENT)
        );
        // Bind binary commands to buttons:
       joystickOperator.getButton(ButtonF310.A).toggleOnTrue(m_intakeCommand);
    }

    public void setTeleopDefaultCommands() {
        // Set default commands (drivetrain, elevator, slide, etc.)
        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_driveCommand);
    }

    public Command getAutonomousCommand() {
        //Return the command for autonomous mode
        return m_autoCommand.getAutonomousCommand();
    }
}