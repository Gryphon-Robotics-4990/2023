package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotionControl;
import frc.robot.vision.VisionController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax armLeft, armRight;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private VisionController m_vision;
    private DigitalInput m_frontLimit, m_backLimit;
    private Timer m_timer;
    private int smartMotionSlot;
    private Servo m_eyebrow1;
    private Servo m_eyebrow2;


    public ArmSubsystem(VisionController vision) {
        //Instantiates two SparkMax motors
        armLeft = new CANSparkMax(Ports.CAN_ARM_LEFT_SPARKMAX, MotorType.kBrushless);
        armRight = new CANSparkMax(Ports.CAN_ARM_RIGHT_SPARKMAX, MotorType.kBrushless);
        m_vision = vision;

        //m_timer = new Timer();
        //m_timer.start();

        //Creates two Limit Switches
        m_frontLimit = new DigitalInput(Ports.DIO_FRONT_LIMIT_SWITCH);
        m_backLimit = new DigitalInput(Ports.DIO_BACK_LIMIT_SWITCH);
        
        m_pidController = armRight.getPIDController();
        m_encoder = armRight.getEncoder();

        m_eyebrow1 = new Servo(0);
        m_eyebrow2 = new Servo(1);
        //Runs configureMotors
        configureMotors();
    }

    private void configureMotors() {
        //Set motor to default 
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
        
        //PID/SmartMotion Stuff 
        m_pidController.setP(MotionControl.ARM_PID.kP);
        m_pidController.setI(MotionControl.ARM_PID.kI);
        m_pidController.setD(MotionControl.ARM_PID.kD);

        armLeft.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE);
        armRight.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE);
        armLeft.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
        armRight.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);

        //Left follows right
        armLeft.follow(armRight, true);
    }

    public boolean isArmAtLimit() {
       // checks if either of the limit switch is triggered 
        return (m_frontLimit.get() || m_backLimit.get());
 

    }
    //Takes in number of rotations for arm (without gear reduction)
    public void moveToPosition(double setPoint) {
        // Velocity and acceleration for Arm feedforward is 0
        //double m_offsetposition = position - 90.0;
        //double m_positionRotations = position*Units.RADIAN.to(Units.REVOLUTION);
        //double m_feedforward = MotionControl.ARM_FEEDFORWARD.calculate(m_offsetposition, 0);
        //m_pidController.setReference(m_positionRotations, ControlType.kPosition, 0, m_feedforward);
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
       }
    

    //Hell if I know :/ 
    public void armPercentOutput(double percent_output) {
        armRight.set(percent_output);
    }

    public double getPosition() {
        return (m_encoder.getPosition());
    }

    public void eyebrowPose(double position)
    {
        m_eyebrow1.set(position);
        m_eyebrow2.set(position);
    }

    @Override
    public void periodic() {
        //System.out.println(m_vision.getLatestResult().hasTargets());
        //System.out.printf("Distance: %f %n", m_vision.getDistanceToTarget());
        //System.out.printf("X: %f %n", m_vision.getTranslationToTarget().getX());
        //System.out.printf("Y: %f %n", m_vision.getTranslationToTarget().getY());
        //System.out.printf("Horizontal Angle: %f %n", m_vision.getHorizontalAngle());
        //System.out.printf("Vertical Angle: %f %n", m_vision.getVerticalAngle());
        //.delay(1);
        SmartDashboard.putBoolean("ArmAtLimit", isArmAtLimit());
        SmartDashboard.putNumber("Arm Position", getPosition());
    }
}