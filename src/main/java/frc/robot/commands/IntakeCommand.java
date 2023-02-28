package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeCommand extends CommandBase {
    //creates the m_intake variable
    private IntakeSubsystem m_intake;

    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    /*@Override
    public void initialize() {
        m_intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.outtake();
    }*/
}
