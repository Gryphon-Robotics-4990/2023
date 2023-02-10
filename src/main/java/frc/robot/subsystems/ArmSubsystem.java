package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMax.ControlType; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotionControl;
import frc.robot.Constants.SubsystemConfig;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax armLeft, armRight;
    private SparkMaxPIDController pidController;

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
    pidController.setReference(position, ControlType.kPosition);
}

//Hell if I know :/ 
public void armPercentOutput(double percent_output) {
    armRight.set(percent_output);
}

}