package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainTranslationCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    private final double m_meters;

    public DrivetrainTranslationCommand(DrivetrainSubsystem drivetrain, double meters) {
        m_drivetrain = drivetrain;
        m_meters = meters;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.driveMeters(m_meters);
    }

    @Override
    public boolean isFinished() {
        return m_drivetrain.inPosition();
    }

}
