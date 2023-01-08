package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionController;
import frc.robot.JoystickF310.*;

import static frc.robot.Constants.*;

public class RobotContainer {
    // The robot's subsystems and commands are defined in this file

    //Create joystick input objects 
    private final JoystickF310 joystickDrive = new JoystickF310(Ports.PORT_JOYSTICK_DRIVE);
    private final JoystickF310 joystickOperator = new JoystickF310(Ports.PORT_JOYSTICK_OPERATOR);

    // Create subsystem objects
    
    //Create command objects
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. **/
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();
        
    }

    private void configureControlBindings() {
        // Set suppliers for commands with joystick axes:
        
    
        // Bind binary commands to buttons:
       
    }

    public void setTeleopDefaultCommands() {
        // Set default commands (drivetrain, elevator, slide, etc.)
        
    }

    public Command getAutonomousCommand() {
        //Return the command for autonomous mode
        return null;
    }
}