package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveUtil;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Calendar;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DrivetrainSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_leftFrontTalon, m_leftRearTalon, m_rightFrontTalon, m_rightRearTalon;
    private final AHRS m_gyro;
    private final DifferentialDriveOdometry m_odometry;

    public DrivetrainSubsystem()
    {
        m_leftFrontTalon = new WPI_TalonSRX(Ports.CAN_DRIVETRAIN_LEFT_FRONT_TALONSRX);
        m_leftRearTalon = new WPI_TalonSRX(Ports.CAN_DRIVETRAIN_LEFT_REAR_TALONSRX);
        m_rightFrontTalon = new WPI_TalonSRX(Ports.CAN_DRIVETRAIN_RIGHT_FRONT_TALONSRX);
        m_rightRearTalon = new WPI_TalonSRX(Ports.CAN_DRIVETRAIN_RIGHT_REAR_TALONSRX);
        
        m_gyro = new AHRS(Ports.SPI_PORT_GYRO);
        //m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), new Pose2d(0, 0, newRotation2d()));
        m_odometry = null;

        configureMotors();

        m_leftFrontTalon.setSelectedSensorPosition(0);
        m_rightFrontTalon.setSelectedSensorPosition(0);
        m_gyro.reset();
    }

    @Override
    public void periodic()
    {
        var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
        m_odometry.update(gyroAngle, m_leftFrontTalon.getSelectedSensorPosition(), m_rightFrontTalon.getSelectedSensorPosition());
    }

    public void drivePO(double left, double right)
    {
        m_leftFrontTalon.set(ControlMode.PercentOutput, left);
        m_rightFrontTalon.set(ControlMode.PercentOutput, right);
    }

    public void drive(double left, double right) {
        m_leftFrontTalon.set(ControlMode.Velocity, left);
        m_rightFrontTalon.set(ControlMode.Velocity, right);
    }

    public double getGyroTilt() {//Figure out which one we're using
        return Math.max(m_gyro.getPitch(), m_gyro.getRoll());
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(m_leftFrontTalon.getSelectedSensorVelocity(), m_rightFrontTalon.getSelectedSensorVelocity());
    }

    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        double leftDistance = (RobotMeasurements.DRIVETRAIN_WHEEL_RADIUS_METERS) * m_leftFrontTalon.getSelectedSensorPosition() * Units.ENCODER_ANGLE.to(Units.RADIAN);
        double rightDistance = (RobotMeasurements.DRIVETRAIN_WHEEL_RADIUS_METERS) * m_rightFrontTalon.getSelectedSensorPosition() * Units.ENCODER_ANGLE.to(Units.RADIAN);
        m_odometry.resetPosition(m_gyro.getRotation2d(), leftDistance, rightDistance, pose);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        drivePO(leftVolts/MotorConfig.TALON_VOLTAGE_CONSTRAINT, rightVolts/MotorConfig.TALON_VOLTAGE_CONSTRAINT);
    }

    public void resetEncoders()
    {
        m_leftFrontTalon.setSelectedSensorPosition(0);
        m_rightFrontTalon.setSelectedSensorPosition(0);
    }

    public double getAverageEncoderDistance()
    {
        return (m_leftFrontTalon.getSelectedSensorPosition() + m_rightFrontTalon.getSelectedSensorPosition())/2.0;
    }

    public void zeroHeading()
    {
        m_gyro.reset();
    }

    public double getHeading()
    {
        return m_gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate()
    {
        return -m_gyro.getRate();
    }

    private void configureMotors()
    {
        //First setup talons with default settings
        m_leftFrontTalon.configFactoryDefault();
        m_leftRearTalon.configFactoryDefault();
        m_rightFrontTalon.configFactoryDefault();
        m_rightRearTalon.configFactoryDefault();

        // TODO figure out whether we need to switch the encoder direction
        m_leftFrontTalon.setSensorPhase(true);
        m_rightFrontTalon.setSensorPhase(true);

        m_rightFrontTalon.setInverted(true);
        
        m_leftRearTalon.follow(m_leftFrontTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);
        m_rightRearTalon.follow(m_rightFrontTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);

        //Setup talon built-in PID
        m_leftFrontTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        m_rightFrontTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        
        //Create config objects
        TalonSRXConfiguration cLeft = new TalonSRXConfiguration(), cRight = new TalonSRXConfiguration();

        //Setup config objects with desired values
        cLeft.slot0 = MotionControl.TEST_DRIVETRAIN_LEFT_PID;
        cRight.slot0 = MotionControl.TEST_DRIVETRAIN_RIGHT_PID;

        //NeutralMode mode = NeutralMode.Brake;
        NeutralMode mode = NeutralMode.Coast;

        //Brake mode so no coasting
        m_leftFrontTalon.setNeutralMode(mode);
        m_leftRearTalon.setNeutralMode(mode);
        m_rightFrontTalon.setNeutralMode(mode);
        m_rightRearTalon.setNeutralMode(mode);

        m_leftFrontTalon.configVoltageCompSaturation(MotorConfig.TALON_VOLTAGE_CONSTRAINT, MotorConfig.TALON_TIMEOUT_MS);
        m_rightFrontTalon.configVoltageCompSaturation(MotorConfig.TALON_VOLTAGE_CONSTRAINT, MotorConfig.TALON_TIMEOUT_MS);

        //Configure talons
        m_leftFrontTalon.configAllSettings(cLeft);
        m_rightFrontTalon.configAllSettings(cRight);
    }
}