package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.DriveUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopArcadeDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private DoubleSupplier m_speedSupplier, m_rotationSupplier;

    public TeleopArcadeDriveCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void setSuppliers(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
        m_speedSupplier = speedSupplier;
        m_rotationSupplier = rotationSupplier;
    }

    @Override
    public void execute() {
        double speed = m_speedSupplier.getAsDouble();
        double rot = m_rotationSupplier.getAsDouble();

        double[] speeds = DriveUtil.arcadeToTankDrive(speed * ARCADE_SPEED_MULTIPLIER, rot * ARCADE_ROTATION_MULTIPLIER);
        // Convert speeds to target speeds in meters per second, and then divide by hypothetical maximum movement speed
        //Proportion of max speed
        //speeds[0] *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        //speeds[1] *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        
        m_drivetrain.drivePO(speeds[0], speeds[1]);
    }

}
