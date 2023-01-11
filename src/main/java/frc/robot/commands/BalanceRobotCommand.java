package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

public class BalanceRobotCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private final PIDController m_pid;
    

    public BalanceRobotCommand(DrivetrainSubsystem drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        // Using an external PID controller b/c the in-built talon PID uses the given encoder
        m_pid = new PIDController(MotionControl.ROBOT_BALANCE_PID.kP, MotionControl.ROBOT_BALANCE_PID.kI, MotionControl.ROBOT_BALANCE_PID.kD);
    }

    @Override
    public void execute() {
        //double tilt = m_drivetrain.getGyroTilt();
        double tilt = 0;
        // Trying to get the gyro tilt to be 0 (PID based on the gyro being the encoder)
        double val = m_pid.calculate(tilt, 0.0);
        m_drivetrain.drivePO(val, val);
    }

}