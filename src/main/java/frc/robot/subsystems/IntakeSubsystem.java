package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intake;

    public IntakeSubsystem() {
        intake = new CANSparkMax(Ports.CAN_INTAKE_SPARKMAX, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        intake.restoreFactoryDefaults();
    }

    public void intakePercentOutput(double percent_output) {
        intake.set(percent_output);
    }

    public double intakeCurrentDraw() {
        return intake.getOutputCurrent();
    }


}
