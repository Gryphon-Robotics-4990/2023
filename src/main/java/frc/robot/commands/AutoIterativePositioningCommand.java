package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.VisionController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.*;

public class AutoIterativePositioningCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    private final VisionController m_vision;
    
    public AutoIterativePositioningCommand(DrivetrainSubsystem drivetrain, VisionController vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // TODO find necessary turret position using VisionController
        double currentAngle = m_drivetrain.getHeading();
        double change = m_vision.getHorizontalAngle();
        // Rotate drivetrain to new angle
        //m_drivetrain.setPosition(currentAngle + change);
    }

}
