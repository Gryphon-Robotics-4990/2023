package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.vision.VisionController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DangerCommand extends CommandBase{
    private final ArmSubsystem m_armsubsystem;
    private final VisionController m_visioncontroller;

    public DangerCommand(ArmSubsystem armsubsystem, VisionController visioncontroller){
        m_armsubsystem = armsubsystem;
        m_visioncontroller = visioncontroller;
        addRequirements(armsubsystem);
    }

    @Override
    public void execute() {
        double distance = m_visioncontroller.getDistanceToTarget(); 
        if (distance > 2){
            distance = 2;
        }
        if (distance <= 0){
            distance = 0;
        }

        double position = distance * 36;
        m_armsubsystem.moveToPosition(position);
    }
}
