package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.vision.VisionController;

public class DrivetrainRotateCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    private final VisionController m_vision;

    public DrivetrainRotateCommand(DrivetrainSubsystem drivetrain, VisionController vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double angle = m_vision.getHorizontalAngle();
        m_drivetrain.rotateDegrees(angle);
    }
    
}
