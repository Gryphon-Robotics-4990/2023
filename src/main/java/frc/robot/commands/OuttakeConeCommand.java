package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class OuttakeConeCommand extends CommandBase {
    private IntakeSubsystem m_intake;
    private double speed = 1;

    public OuttakeConeCommand(IntakeSubsystem intake) {
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
