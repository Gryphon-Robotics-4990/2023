package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Grabber extends SubsystemBase {
    //Creates motors and pidcontroller
    private CANSparkMax grabberLeft, grabberRight;
    private SparkMaxPIDController pidController;

    public Grabber() {
        //Instantiates two SparkMax motors
        grabberLeft = new CANSparkMax(Ports.CAN_GRABBER_LEFT_SPARKMAX, MotorType.kBrushless);
        grabberRight = new CANSparkMax(Ports.CAN_GRABBER_RIGHT_SPARKMAX, MotorType.kBrushless);

        //Runs connfigureMotoes
        configureMotors();
    }

    private void configureMotors() {
        //Sets motors to defaults
        grabberLeft.restoreFactoryDefaults();
        grabberRight.restoreFactoryDefaults();

        //Inverts right motor and has left follow right
        grabberRight.setInverted(true);
        grabberLeft.follow(grabberRight);

        //creates PID controller    
        pidController = grabberRight.getPIDController();
        
        //idk man
        pidController.setP(MotionControl.GRABBER_PID.kP);
        pidController.setI(MotionControl.GRABBER_PID.kI);
        pidController.setD(MotionControl.GRABBER_PID.kD);
    }

    public void grab() {
        //Moves motors to grab
        //Placeholder rotation
        pidController.setReference(1.0, ControlType.kPosition);
    }

    public void ungrab() {
        //Moves motor to nuetral position
        //Dependant on whether 0 is closed or open
        pidController.setReference(0, ControlType.kPosition);
    }
}
