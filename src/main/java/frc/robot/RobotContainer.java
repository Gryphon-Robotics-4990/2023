package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.JoystickF310.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.AutoPositioningCommand;
import frc.robot.vision.AutoPositioningController;
import frc.robot.vision.VisionController;
import frc.robot.commands.ArmEStopCommand;
import frc.robot.commands.ArmIntakeConeCommand;
import frc.robot.commands.ArmIntakeCubeCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmNeutralCommand;
import frc.robot.commands.ArmPercentOutputCommand;
import frc.robot.commands.ArmPlaceCommand;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.AutoCenterCommand;
import frc.robot.commands.AutoCenterCubeCommand;
import frc.robot.commands.AutoLeftCommand;
import frc.robot.commands.AutoLeftCubeCommand;
import frc.robot.commands.AutoRightCommand;
import frc.robot.commands.AutoRightCubeCommand;
import frc.robot.commands.DangerCommand;
import frc.robot.commands.DrivetrainTranslationCommand;
import frc.robot.commands.EyebrowPositionCommand;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.LowerCubeCommand;
import frc.robot.commands.MiddleCubeCommand;
import frc.robot.commands.OuttakeCubeCommand;
import frc.robot.commands.SlowArcadeDriveCommand;
import frc.robot.commands.IntakeConeCommand;
import frc.robot.commands.OuttakeConeCommand;
import frc.robot.commands.TeleopArcadeDriveCommand;
import frc.robot.commands.TopCubeCommand;

import static frc.robot.Constants.*;

import java.util.function.Supplier;

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
    //private final AutoCenterCommand m_autoCenterCommand = new AutoCenterCommand(m_drivetrain);
    //private final AutoLeftCommand m_autoLeftCommand = new AutoLeftCommand(m_drivetrain);
    //private final AutoRightCommand m_autoRightCommand = new AutoRightCommand(m_drivetrain);
    private final TeleopArcadeDriveCommand m_teleopDriveCommand = new TeleopArcadeDriveCommand(m_drivetrain);
    private final SlowArcadeDriveCommand m_slowDriveCommand = new SlowArcadeDriveCommand(m_drivetrain);
    private final ArmManualCommand m_armManualCommand = new ArmManualCommand(m_arm);
    private final OuttakeCubeCommand m_outtakeCubeCommand = new OuttakeCubeCommand(m_intake);
    private final OuttakeConeCommand m_outtakeConeCommand = new OuttakeConeCommand(m_intake);
    private final EyebrowPositionCommand m_eyebrowPositionCommand = new EyebrowPositionCommand(m_arm);
    private SendableChooser<Supplier<Command>> m_chooser = new SendableChooser<>();
    private SendableChooser<Command> m_cubeShelfChooser = new SendableChooser<>();

    private final ArmIntakeConeCommand m_armConeIntakeCommand = new ArmIntakeConeCommand(m_arm);
    private final ArmIntakeCubeCommand m_armCubeIntakeCommand = new ArmIntakeCubeCommand(m_arm);
    private final LowerCubeCommand m_lowerCubeCommand = new LowerCubeCommand(m_arm);
    private final MiddleCubeCommand m_middleCubeCommand = new MiddleCubeCommand(m_arm);
    private final TopCubeCommand m_topCubeCommand = new TopCubeCommand(m_arm);
    private final ArmNeutralCommand m_armNeutralCommand = new ArmNeutralCommand(m_arm);
    private final ArmEStopCommand m_armStopCommand = new ArmEStopCommand(m_arm);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. **/
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();
        
        m_cubeShelfChooser.setDefaultOption("Top Cube", new TopCubeCommand(m_arm));
        m_cubeShelfChooser.addOption("Middle Cube", new MiddleCubeCommand(m_arm));
        m_cubeShelfChooser.addOption("Lower Cube", new LowerCubeCommand(m_arm));
        Shuffleboard.getTab("Tab 1").add("Choose Auto Cube Shelf", m_cubeShelfChooser);

        m_chooser.setDefaultOption("Center Start", () -> new AutoCenterCubeCommand(m_drivetrain, m_arm, m_intake, m_cubeShelfChooser));
        m_chooser.addOption("Left Start", () -> new AutoLeftCubeCommand(m_drivetrain, m_arm, m_intake, m_cubeShelfChooser));
        m_chooser.addOption("Right Start", () -> new AutoRightCubeCommand(m_drivetrain, m_arm, m_intake, m_cubeShelfChooser));
        Shuffleboard.getTab("Tab 1").add("Choose Auto Path", m_chooser);
    }

    private void configureControlBindings() {
        // Set suppliers for commands with joystick axes:
        m_teleopDriveCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_THROTTLE_EXPONENT),
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_TURNING_EXPONENT)
        );

        m_slowDriveCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), SLOW_JOYSTICK_THROTTLE_EXPONENT),
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_TURNING_EXPONENT)
        );
        
        m_armManualCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.JoystickLeftY), 1)
        );

        m_eyebrowPositionCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.TriggerRight),1)
        );

        // Bind binary commands to buttons:
        joystickOperator.getButton(ButtonF310.B).toggleOnTrue(m_outtakeConeCommand);
        joystickOperator.getButton(ButtonF310.X).toggleOnTrue(m_outtakeCubeCommand);

        joystickOperator.getButton(POVF310.Top).onTrue(m_topCubeCommand);
        joystickOperator.getButton(POVF310.Right).onTrue(m_middleCubeCommand);
        joystickOperator.getButton(POVF310.Bottom).onTrue(m_lowerCubeCommand);
        joystickOperator.getButton(POVF310.Left).onTrue(m_armCubeIntakeCommand);
        joystickOperator.getButton(ButtonF310.Y).onTrue(m_armNeutralCommand);
        joystickOperator.getButton(ButtonF310.BumperLeft).onTrue(m_armManualCommand);
        joystickDrive.getButton(ButtonF310.BumperLeft).onTrue(m_armNeutralCommand);
        joystickOperator.getButton(POVF310.BottomLeft).onTrue(m_armConeIntakeCommand);
        joystickOperator.getButton(ButtonF310.BumperRight).onTrue(m_eyebrowPositionCommand);
        joystickOperator.getButton(ButtonF310.A).onTrue(m_armStopCommand);

        joystickDrive.getButton(ButtonF310.BumperRight).toggleOnTrue(m_slowDriveCommand);
    }

    public void setTeleopDefaultCommands() {
        // Set default commands (drivetrain, elevator, slide, etc.)
        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopDriveCommand);
        CommandScheduler.getInstance().setDefaultCommand(m_arm, m_armManualCommand);
    }

    

    public Command getAutonomousCommand() {
        //Return the command for autonomous mode
        return(m_chooser.getSelected().get());
    }
}
