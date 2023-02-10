package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TeleopArcadeDriveCommand;

public class ArmManualCommand extends CommandBase {
    private final ArmSubsystem m_armsubsystem;
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
        m_armsubsystem.armPercentOutput(speed);
    }
   
}