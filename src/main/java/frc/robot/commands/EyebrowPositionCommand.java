package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class EyebrowPositionCommand extends CommandBase{
    private final ArmSubsystem m_armsubsystem;
    private DoubleSupplier m_joystickSupplier; 

    public EyebrowPositionCommand(ArmSubsystem armSubsystem)
    {
        m_armsubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    public void setSuppliers(DoubleSupplier joystickSupplier) {
        m_joystickSupplier = joystickSupplier;
    }

    @Override
    public void execute()
    {
        double position = m_joystickSupplier.getAsDouble();
        m_armsubsystem.eyebrowPose(position);
    }
}
