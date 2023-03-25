package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeCubeCommand extends CommandBase{
    private IntakeSubsystem m_intake;
    private double speed = 0.4;

    public OuttakeCubeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.intakePercentOutput(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.intakePercentOutput(0.0);
    }
}
