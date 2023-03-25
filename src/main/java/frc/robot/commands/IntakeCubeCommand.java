package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeCubeCommand extends CommandBase {
    private IntakeSubsystem m_intake;
    private double speed = 0.2;

    public IntakeCubeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.intakePercentOutput(-1 * speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intakePercentOutput(0.0);
    }
}
