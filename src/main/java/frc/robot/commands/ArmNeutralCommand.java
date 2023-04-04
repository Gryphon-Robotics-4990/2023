package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmNeutralCommand extends CommandBase{
    private final ArmSubsystem m_armsubsystem;
    
    public ArmNeutralCommand(ArmSubsystem armsubsystem) {
        m_armsubsystem = armsubsystem;
        addRequirements(armsubsystem);
    }

    @Override
    public void initialize() {
        m_armsubsystem.moveToPosition(0);
    }
}
