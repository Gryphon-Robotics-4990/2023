package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmManualCommand extends CommandBase {
    private final ArmSubsystem m_armsubsystem;
    private double mult = 0.5;
    private DoubleSupplier m_joystickSupplier; 
    
    public ArmManualCommand(ArmSubsystem armsubsystem) {
        m_armsubsystem = armsubsystem;
        addRequirements(armsubsystem);
    }

    public void setSuppliers(DoubleSupplier joystickSupplier) {
        m_joystickSupplier = joystickSupplier;
    }

    @Override
    public void execute() {
        double speed = m_joystickSupplier.getAsDouble(); 
        // If the does not arm trips the limit switch move the armm, otherwise set output to 0
        //if (!m_armsubsystem.isArmAtLimit()) {
        
        // Quick hack to get arm to hold position while we don't have arm PID
        if (speed != 0.0) {
            m_armsubsystem.armPercentOutput(speed*mult);
        } else {
            m_armsubsystem.armPercentOutput(0.01);
        }
        
        // } else {
        //     System.out.println("Stopped Arm");
        //     m_armsubsystem.armPercentOutput(0);
        // }
    }
}