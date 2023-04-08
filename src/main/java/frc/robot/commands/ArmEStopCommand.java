package frc.robot.commands;
import frc.robot.Constants.RobotMeasurements;
import frc.robot.Constants.Units;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmEStopCommand extends CommandBase {
    private final ArmSubsystem m_armsubsystem;
    private double mult = 36.0;
    private DoubleSupplier m_joystickSupplier; 
    
    public ArmEStopCommand(ArmSubsystem armsubsystem) {
        m_armsubsystem = armsubsystem;
        addRequirements(armsubsystem);
    }

    @Override
    public void execute() {
        m_armsubsystem.armPercentOutput(0);
    }
}