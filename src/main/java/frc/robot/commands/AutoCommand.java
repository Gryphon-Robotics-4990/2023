package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.MotionControl;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoCommand {
    private final DrivetrainSubsystem m_drivetrain;

    public AutoCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
    }

    public RamseteCommand getAutonomousCommand() {
        // Create voltage constraint to ensure we don't accelerate too fast

        // var autoVoltageConstraint = 
        // new DifferentialDriveVoltageConstraint(
        // new SimpleMotorFeedforward(
            // MotionControl.ksVolts,
            // MotionControl.kvVoltSecondsPerMeter,
            // MotionControl.kaVoltSecondsSquaredPerMeter),
        // MotionControl.kDriveKinematics,
        // 10);

        // Create config for trajectory
        TrajectoryConfig config = 
            new TrajectoryConfig(
                    MotionControl.kMaxSpeedMetersPerSecond,
                    MotionControl.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(MotionControl.kDriveKinematics);
            // Apply the voltage constraint
            // .addConstraint(autoVoltageConstraint);

        // Placeholder positions
        Trajectory exampleTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
        
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
                m_drivetrain::getPose,
                new RamseteController(MotionControl.kRamseteB, MotionControl.kRamseteZeta),
                new SimpleMotorFeedforward(
                    MotionControl.ksVolts,
                    MotionControl.kvVoltSecondsPerMeter),
                MotionControl.kDriveKinematics,
                m_drivetrain::getWheelSpeeds,
                new PIDController(MotionControl.TEST_DRIVETRAIN_LEFT_PID.kP, 0, 0),
                new PIDController(MotionControl.TEST_DRIVETRAIN_RIGHT_PID.kP, 0, 0),
                // RamseteCommand passes volts to the callback
                m_drivetrain::tankDriveVolts,
                m_drivetrain);
                
        // Reset odometry to the starting pose of the trajectory
        m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end
        return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
    }
}
