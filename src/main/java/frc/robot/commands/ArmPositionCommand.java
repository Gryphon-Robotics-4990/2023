package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends CommandBase{
    private final ArmSubsystem m_armsubsystem;
    private static int m_intPosition;
    private double armPos;

    public ArmPositionCommand(ArmSubsystem armsubsystem) {
        m_armsubsystem = armsubsystem;
        addRequirements(armsubsystem);
    }

    public void setPosition(int position) {
        m_intPosition = position;
    }

    @Override
    public void initialize() {
        if (m_intPosition == 0)
        {
            armPos = 0;
        } else if (m_intPosition == 1)
        {
            armPos = 22;
        } else if (m_intPosition == 2)
        {
            armPos = 60;
        } else if (m_intPosition == 3)
        {
            armPos = 72;
        } else if (m_intPosition == -1)
        {
            armPos = -60;
        } else if (m_intPosition == -2)
        {
            armPos = -72;
        }
        m_armsubsystem.moveToPosition(armPos);
    }
}
