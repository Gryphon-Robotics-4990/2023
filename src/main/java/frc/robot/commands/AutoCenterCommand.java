package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MotionControl;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.*;

public class AutoCenterCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;

    public AutoCenterCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
    }

    public SequentialCommandGroup getAutonomousCommand() {
        // Create voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = 
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    MotionControl.ksVolts,
                    MotionControl.kvVoltSecondsPerMeter,
                    MotionControl.kaVoltSecondsSquaredPerMeter),
                MotionControl.kDriveKinematics,
                SubsystemConfig.AUTO_MAX_VOLTAGE);

        // Create config for trajectory
        TrajectoryConfig config = 
            new TrajectoryConfig(
                    MotionControl.kMaxSpeedMetersPerSecond,
                    MotionControl.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(MotionControl.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        // Placeholder positions
        /*Trajectory centerStart = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),
                new Pose2d(1.5, 0, new Rotation2d(0)),
                // Pass config
                config);*/
        
        PathPlannerTrajectory centerStart = PathPlanner.loadPath("Center_Start", new PathConstraints(2, 1));
        
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                centerStart,
                m_drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(
                    MotionControl.ksVolts,
                    MotionControl.kvVoltSecondsPerMeter),
                MotionControl.kDriveKinematics,
                m_drivetrain::getWheelSpeeds,
                new PIDController(MotionControl.drivekP, 0, MotionControl.drivekD),
                new PIDController(MotionControl.drivekP, 0, MotionControl.drivekD),
                // RamseteCommand passes volts to the callback
                m_drivetrain::tankDriveVolts,
                m_drivetrain);
                
        // Reset odometry to the starting pose of the trajectory
        m_drivetrain.resetOdometry(centerStart.getInitialPose());

        // Run path following command, then stop at the end
        return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
    }
}
