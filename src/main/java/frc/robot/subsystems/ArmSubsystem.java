package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMax.ControlType; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotionControl;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax armLeft, armRight;
    private SparkMaxPIDController pidController;

    public ArmSubsystem() {
        //Instantiates two SparkMax motors
        armLeft = new CANSparkMax(Ports.CAN_GRABBER_LEFT_SPARKMAX, MotorType.kBrushless);
        armRight = new CANSparkMax(Ports.CAN_GRABBER_RIGHT_SPARKMAX, MotorType.kBrushless);

        //Runs configureMotors
        configureMotors();
    }

    private void configureMotors() {
        //Set motor to default 
        armLeft.restoreFactoryDefaults();
        armRight.restoreFactoryDefaults();
        
        //PID Stuff 
        pidController.setP(MotionControl.ARM_PID.kP);
        pidController.setI(MotionControl.ARM_PID.kI);
        pidController.setD(MotionControl.ARM_PID.kD);

        //Inverts right motor; left follows right
        armRight.setInverted(true);
        armLeft.follow(armRight); 

    }
    //Takes in some degree out of 360 
    public void moveToPosition(double position) {
        //This doesn't take into account gear ratio 
        //IN ROTATIONS
        // Velociity and acceleration for Arm feedforward is 0
        pidController.setReference(position, ControlType.kPosition, 0, MotionControl.DRIVETRAIN_FEEDFORWARD.calculate(position, 0));
       }
    

    //Hell if I know :/ 
    public void armPercentOutput(double percent_output) {
        armRight.set(percent_output);
    }

}