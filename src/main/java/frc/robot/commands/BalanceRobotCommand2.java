package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceRobotCommand2 extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    private final double tolerance;

    public BalanceRobotCommand2(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        tolerance = 4.0;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (Math.abs(m_drivetrain.getGyroTilt()) > tolerance) {
            m_drivetrain.driveVelocity(m_drivetrain.getGyroTilt() * 10, m_drivetrain.getGyroTilt() * 10);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drivePO(0, 0);
    }
}
