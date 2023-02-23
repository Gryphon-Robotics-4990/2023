package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.vision.VisionController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.*;

public class IterativePositioningCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    private final VisionController m_vision;
    private RamseteController controller;
    
    public IterativePositioningCommand(DrivetrainSubsystem drivetrain, VisionController vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        controller = new RamseteController();
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Find necessary change in pose using VisionController
        Translation2d translationChange = m_vision.getTranslationToTarget();
        double xChange = translationChange.getX();
        double yChange = translationChange.getY();
        double angleChange = m_vision.getHorizontalAngle();
        // Calculate goal pose
        Pose2d currentRobotPose = m_drivetrain.getPose();
        double currentX = currentRobotPose.getX();
        double currentY = currentRobotPose.getY();
        double currentAngle = m_drivetrain.getHeading();
        Pose2d goalPose = new Pose2d(currentX+xChange, currentY+yChange, new Rotation2d(currentAngle+angleChange));
        // Calculate and pass in speeds for drivetrain
        ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goalPose, 0, 0);
        DifferentialDriveWheelSpeeds wheelSpeeds = MotionControl.kDriveKinematics.toWheelSpeeds(adjustedSpeeds);
        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;
        left *= Units.METERS_PER_SECOND.to(Units.ENCODER_VELOCITY_UNIT);
        right *= Units.METERS_PER_SECOND.to(Units.ENCODER_VELOCITY_UNIT);
        m_drivetrain.drive(left, right);
    }

}
