package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.JoystickF310.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.AutoPositioningCommand;
import frc.robot.vision.AutoPositioningController;
import frc.robot.vision.VisionController;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPercentOutputCommand;
import frc.robot.commands.ArmPlaceCommand;
import frc.robot.commands.AutoCenterCommand;
import frc.robot.commands.AutoLeftCommand;
import frc.robot.commands.AutoRightCommand;
import frc.robot.commands.DangerCommand;
import frc.robot.commands.EyebrowPositionCommand;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.IterativePositioningCommand;
import frc.robot.commands.OuttakeCubeCommand;
import frc.robot.commands.IntakeConeCommand;
import frc.robot.commands.OuttakeConeCommand;
import frc.robot.commands.TeleopArcadeDriveCommand;

import static frc.robot.Constants.*;

public class RobotContainer {
    // The robot's subsystems and commands are defined in this file

    //Create joystick input objects 
    private final JoystickF310 joystickDrive = new JoystickF310(Ports.PORT_JOYSTICK_DRIVE);
    private final JoystickF310 joystickOperator = new JoystickF310(Ports.PORT_JOYSTICK_OPERATOR);

    private final VisionController m_vision = new VisionController();

    // Create subsystem objects
    private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem(m_vision);
    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    //private final AutoPositioningController m_autoPositioningController = new AutoPositioningController(m_drivetrain, m_vision);
    //private final AutoPositioningCommand m_autoPositioningCommand = new AutoPositioningCommand(m_autoPositioningController);
    private final IterativePositioningCommand m_iterativePositioningCommand = new IterativePositioningCommand(m_drivetrain, m_vision);

    //Create command objects
    private final AutoCenterCommand m_autoCenterCommand = new AutoCenterCommand(m_drivetrain);
    private final AutoLeftCommand m_autoLeftCommand = new AutoLeftCommand(m_drivetrain);
    private final AutoRightCommand m_autoRightCommand = new AutoRightCommand(m_drivetrain);
    private final TeleopArcadeDriveCommand m_driveCommand = new TeleopArcadeDriveCommand(m_drivetrain);
    private final ArmManualCommand m_armManualCommand = new ArmManualCommand(m_arm);
    private final ArmPercentOutputCommand m_armPercentOutputCommand = new ArmPercentOutputCommand(m_arm);
    private final ArmPlaceCommand m_armPlaceCommand = new ArmPlaceCommand(m_arm);
    private final IntakeCubeCommand m_intakeCubeCommand = new IntakeCubeCommand(m_intake);
    private final OuttakeCubeCommand m_outtakeCubeCommand = new OuttakeCubeCommand(m_intake);
    private final IntakeConeCommand m_intakeConeCommand = new IntakeConeCommand(m_intake);
    private final OuttakeConeCommand m_outtakeConeCommand = new OuttakeConeCommand(m_intake);
    private final DangerCommand m_dangerCommand = new DangerCommand(m_arm, m_vision);
    private final EyebrowPositionCommand m_eyebrowPositionCommand = new EyebrowPositionCommand(m_arm);
    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. **/
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();

        m_chooser.setDefaultOption("Center Start", m_autoCenterCommand.getAutonomousCommand());
        m_chooser.addOption("Left Start", m_autoLeftCommand.getAutonomousCommand());
        m_chooser.addOption("Right Start", m_autoRightCommand.getAutonomousCommand());
        Shuffleboard.getTab("Tab 3").add("Choose Auto Path", m_chooser);
    }

    private void configureControlBindings() {
        // Set suppliers for commands with joystick axes:
        m_driveCommand.setSuppliers(
            () -> joystickDrive.getRawAxis(AxisF310.JoystickLeftY),
            () -> joystickDrive.getRawAxis(AxisF310.JoystickRightX)
        );
        
        m_armManualCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.JoystickLeftY), 1)
        );

        m_eyebrowPositionCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.TriggerRight),1)
        );

        // Bind binary commands to buttons:
        joystickOperator.getButton(ButtonF310.B).toggleOnTrue(m_intakeCubeCommand);
        joystickOperator.getButton(ButtonF310.X).toggleOnTrue(m_outtakeCubeCommand);
        joystickOperator.getButton(ButtonF310.A).toggleOnTrue(m_intakeConeCommand);
        joystickOperator.getButton(ButtonF310.Y).toggleOnTrue(m_outtakeConeCommand);
        //joystickOperator.getButton(ButtonF310.).toggleOnTrue(m_armPlaceCommand);
    }

    public void setTeleopDefaultCommands() {
        // Set default commands (drivetrain, elevator, slide, etc.)
        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_driveCommand);
        CommandScheduler.getInstance().setDefaultCommand(m_arm, m_armManualCommand);

    }

    public Command getAutonomousCommand() {
        //Return the command for autonomous mode
        return(m_chooser.getSelected());
    }
}
